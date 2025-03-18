#ifndef __WIN_BLE_PORT_HPP__
#define __WIN_BLE_PORT_HPP__

#define ENABLE_WIN_BLUETOOTH
#ifdef ENABLE_WIN_BLUETOOTH

#include "impls/Port.hpp"
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Devices.Enumeration.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Storage.Streams.h>
#include <array>
#include <atomic>
#include <regex>

namespace transport {

using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Devices::Enumeration;

class BluetoothPort : public Port {
public:
    using SharedPtr = std::shared_ptr<BluetoothPort>;
    constexpr static int MAX_RETRIES = 3;
    constexpr static int RECONNECT_INTERVAL_MS = 2000;

    /**
     * @brief 使用标准类型构造BLE端口
     * @param serviceUUID 服务UUID字符串，格式：xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
     * @param txCharUUID 发送特征UUID字符串
     * @param rxCharUUID 接收特征UUID字符串
     */
    BluetoothPort(std::string portName,
           std::string serviceUUID,
           std::string txCharUUID,
           std::string rxCharUUID,
           uint32_t groupId = 0,
           uint32_t portId = 0,
           std::string passwd = "")
        : Port(std::move(portName), groupId, portId, std::move(passwd)),
          m_serviceUUID(serviceUUID),
          m_txUUID(txCharUUID),
          m_rxUUID(rxCharUUID),
          m_targetDeviceName(winrt::to_hstring(portName))
    {
        m_workerThread = std::jthread(&BluetoothPort::WorkerProc, this);
    }

    ~BluetoothPort() {
        m_stopRequested = true;
        if(m_workerThread.joinable()) m_workerThread.join();
        DisconnectDevice();
    }

    bool reinit() override {
        std::lock_guard lock(m_deviceMutex);
        m_port_is_available = false;
        return ConnectDevice(true);
    }

private:
    // BLE特征标识
    winrt::guid m_serviceUUID;
    winrt::guid m_txUUID;
    winrt::guid m_rxUUID;
    winrt::hstring m_targetDeviceName;

    // BLE连接状态
    BluetoothLEDevice m_device{nullptr};
    GattDeviceService m_service{nullptr};
    GattCharacteristic m_txChar{nullptr};
    GattCharacteristic m_rxChar{nullptr};

    // 线程管理
    std::jthread m_workerThread;
    std::atomic_bool m_stopRequested{false};
    std::mutex m_deviceMutex;
    std::atomic_int m_retryCount{0};

    void WorkerProc() {
        while (!m_stopRequested) {
            if (!m_port_is_available) {
                TryConnectDevice();
            }

            if (m_port_is_available) {
                ProcessSendQueue();
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(m_port_is_available ? 10 : 500));
        }
    }

    void TryConnectDevice() {
        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            if (ConnectDevice()) {
                UpdatePortStatus(true);
                return;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(RECONNECT_INTERVAL_MS));
        }
        LOGERROR("Failed to connect after %d attempts", MAX_RETRIES);
        UpdatePortStatus(false);
    }

    bool ConnectDevice(bool forceReconnect = false) {
        try {
            auto device = FindBLEDevice().get();
            if (!device) return false;

            std::lock_guard lock(m_deviceMutex);
            if (forceReconnect) {
                DisconnectDevice();
            }

            return InitializeService(device).get() && 
                   InitializeCharacteristics().get();
        } catch (const winrt::hresult_error& e) {
            LOGERROR("BLE连接失败: 0x%X %ls", 
                    e.code(), e.message().c_str());
            return false;
        }
    }

    IAsyncOperation<BluetoothLEDevice> FindBLEDevice() {
        auto selector = BluetoothLEDevice::GetDeviceSelector();
        auto devices = co_await DeviceInformation::FindAllAsync(selector); // 使用全名
        
        for (auto&& device : devices) {
            if (device.Name() == m_targetDeviceName) {
                auto bleDevice = co_await BluetoothLEDevice::FromIdAsync(device.Id());
                if (bleDevice && bleDevice.ConnectionStatus() == 
                    BluetoothConnectionStatus::Connected) {
                    co_return bleDevice;
                }
            }
        }
        co_return nullptr;
    }

    IAsyncOperation<bool> InitializeService(BluetoothLEDevice device) {
        co_await winrt::resume_background();
        
        try {
            auto services = co_await device.GetGattServicesAsync();
            if (services.Status() != GattCommunicationStatus::Success) {
                LOGERROR("获取服务失败: %d", (int)services.Status());
                co_return false;
            }

            for (auto&& service : services.Services()) {
                if (service.Uuid() == m_serviceUUID) {
                    m_service = service;
                    co_return true;
                }
            }
            LOGERROR("未找到指定服务");
            co_return false;
        } catch (...) {
            co_return false;
        }
    }

    IAsyncOperation<bool> InitializeCharacteristics() {
        co_await winrt::resume_background();
        
        try {
            // 获取特征值
            auto txResult = co_await m_service.GetCharacteristicsForUuidAsync(m_txUUID);
            auto rxResult = co_await m_service.GetCharacteristicsForUuidAsync(m_rxUUID);
            
            if (txResult.Status() != GattCommunicationStatus::Success) {
                LOGERROR("发送特征获取失败: %d", (int)txResult.Status());
                co_return false;
            }
            if (rxResult.Status() != GattCommunicationStatus::Success) {
                LOGERROR("接收特征获取失败: %d", (int)rxResult.Status());
                co_return false;
            }

            m_txChar = txResult.Characteristics().GetAt(0);
            m_rxChar = rxResult.Characteristics().GetAt(0);

            // 订阅通知
            auto status = co_await m_rxChar.WriteClientCharacteristicConfigurationDescriptorAsync(
                GattClientCharacteristicConfigurationDescriptorValue::Notify);
            if (status != GattCommunicationStatus::Success) {
                LOGERROR("通知订阅失败: %d", (int)status);
                co_return false;
            }

            m_rxChar.ValueChanged({this, &BluetoothPort::OnCharacteristicChanged});
            co_return true;
        } catch (...) {
            co_return false;
        }
    }

    void ProcessSendQueue() {
        BufferWithID buffer;
        while (popOneBuffer(buffer)) {
            if (!SendData(buffer)) {
                LOGWARN("数据发送失败，重新入队");
                pushOneBuffer(buffer);
                break;
            }
        }
    }

    bool SendData(BufferWithID& buffer) {
        try {
            DataWriter writer;
            writer.WriteBytes(winrt::array_view<const uint8_t>(buffer.buffer.data(), 
                                buffer.buffer.data() + buffer.buffer.size()));
            
            auto result = m_txChar.WriteValueAsync(writer.DetachBuffer()).get();
            return result == GattCommunicationStatus::Success;
        } catch (...) {
            return false;
        }
    }

    void OnCharacteristicChanged(GattCharacteristic const&, 
                               GattValueChangedEventArgs const& args) {
        try {
            DataReader reader = DataReader::FromBuffer(args.CharacteristicValue());
            auto len = reader.UnconsumedBufferLength();
            std::vector<uint8_t> data(len);
            reader.ReadBytes(data);

            Buffer buffer;
            buffer.copy(data.data(), data.size());
            ID id = ParsePacketID(data);
            recvOnePackage(id, buffer);
        } catch (...) {
            LOGERROR("数据处理异常");
        }
    }

    void UpdatePortStatus(bool connected) {
        std::lock_guard lock(m_deviceMutex);
        m_port_is_available = connected;
        if (m_port_status) {
            m_port_status->status = connected ? 
                PortStatus::Available : PortStatus::Unavailable;
        }
    }

    void DisconnectDevice() {
        if (m_rxChar) {
            m_rxChar.ValueChanged(nullptr);
        }
        if (m_device) {
            m_device.Close();
            m_device = nullptr;
        }
        m_service = nullptr;
        m_txChar = nullptr;
        m_rxChar = nullptr;
    }

    // 协议解析示例（需根据实际协议实现）
    ID ParsePacketID(const std::vector<uint8_t>& data) {
        if (data.size() < sizeof(ID)) return 0;
        return *reinterpret_cast<const ID*>(data.data());
    }
};

} // namespace transport

#endif // ENABLE_WIN_BLUETOOTH
#endif // __WIN_BLE_PORT_HPP__