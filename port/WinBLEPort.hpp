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

using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Devices::Enumeration;

namespace transport {

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
    BluetoothPort(std::string portName, uint32_t groupId = 0, uint32_t portId = 0, std::string passwd = "")
        : Port(portName, groupId, portId, passwd),
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
    winrt::hstring m_targetDeviceName;

    // BLE连接状态
    BluetoothLEDevice m_device{nullptr};
    std::unordered_map<uint16_t, GattCharacteristic> m_txCharacteristics;
    std::unordered_map<uint16_t, GattCharacteristic> m_rxCharacteristics;

    // 线程管理
    std::jthread m_workerThread;
    std::atomic_bool m_stopRequested{false};
    std::mutex m_deviceMutex;
    std::atomic_int m_retryCount{0};

    // UUID转换工具
    static uint16_t ExtractShortUUID(const winrt::guid& uuid) {
        // 检查标准蓝牙UUID格式：0000xxxx-0000-1000-8000-00805f9b34fb
        const uint8_t baseUUID[] = {0x00,0x00,0x10,0x00,0x80,0x00,0x00,0x80,0x5F,0x9B,0x34,0xFB};
        if(memcmp(uuid.Data4, baseUUID, sizeof(baseUUID)) == 0 &&
           uuid.Data3 == 0x0000 &&
           (uuid.Data1 & 0xFFFF0000) == 0x00000000) {
            return static_cast<uint16_t>(uuid.Data1 & 0xFFFF);
        }
        return 0; // 非标准短UUID
    }

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

            return InitializeServices(device).get();
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

    IAsyncOperation<bool> InitializeServices(BluetoothLEDevice device) {
        co_await winrt::resume_background();
        
        try {
            // 清空特征映射
            m_txCharacteristics.clear();
            m_rxCharacteristics.clear();

            // 获取所有服务
            auto servicesResult = co_await device.GetGattServicesAsync();
            if (servicesResult.Status() != GattCommunicationStatus::Success) {
                LOGERROR("获取服务失败: %d", (int)servicesResult.Status());
                co_return false;
            }

            // 遍历所有服务
            for (auto&& service : servicesResult.Services()) {
                auto charsResult = co_await service.GetCharacteristicsAsync();
                if (charsResult.Status() != GattCommunicationStatus::Success) {
                    continue;
                }

                // 处理特征
                for (auto&& characteristic : charsResult.Characteristics()) {
                    auto properties = characteristic.CharacteristicProperties();
                    uint16_t shortUUID = ExtractShortUUID(characteristic.Uuid());

                    // 处理发送特征
                    if ((properties & GattCharacteristicProperties::Write) != GattCharacteristicProperties::None || 
                        (properties & GattCharacteristicProperties::WriteWithoutResponse) != GattCharacteristicProperties::None) {
                        m_txCharacteristics.emplace(shortUUID, characteristic);
                    }

                    // 处理接收特征
                    if ((properties & GattCharacteristicProperties::Notify) != GattCharacteristicProperties::None) {
                        // 订阅通知
                        auto status = co_await characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                            GattClientCharacteristicConfigurationDescriptorValue::Notify);
                        
                        if (status == GattCommunicationStatus::Success) {
                            // 绑定带ID的回调
                            characteristic.ValueChanged(
                                [this, shortUUID](GattCharacteristic const&, GattValueChangedEventArgs const& args) {
                                    OnCharacteristicChanged(shortUUID, args);
                                });
                            m_rxCharacteristics.emplace(shortUUID, characteristic);
                        }
                    }
                }
            }

            co_return !m_txCharacteristics.empty() && !m_rxCharacteristics.empty();
        } catch (...) {
            co_return false;
        }
    }

    void ProcessSendQueue() {
        BufferWithID buffer;
        while (popOneBuffer(buffer)) {
            uint16_t id = unmask(buffer.id);
            if (!SendData(id, buffer.buffer)) {
                LOGWARN("数据发送失败，重新入队");
                pushOneBuffer(buffer);
                break;
            }
        }
    }

    bool SendData(uint16_t id, const Buffer& data) {
        std::lock_guard lock(m_deviceMutex);
        auto it = m_txCharacteristics.find(id);
        if (it == m_txCharacteristics.end()) {
            LOGERROR("未找到对应ID的特征: 0x%04X", id);
            return false;
        }

        try {
            DataWriter writer;
            writer.WriteBytes(winrt::array_view(data.data(), data.data() + data.size()));
            auto result = it->second.WriteValueAsync(writer.DetachBuffer()).get();
            return result == GattCommunicationStatus::Success;
        } catch (...) {
            return false;
        }
    }

    void OnCharacteristicChanged(uint16_t id, 
                               GattValueChangedEventArgs const& args) {
        try {
            DataReader reader = DataReader::FromBuffer(args.CharacteristicValue());
            auto len = reader.UnconsumedBufferLength();
            std::vector<uint8_t> data(len);
            reader.ReadBytes(data);

            Buffer buffer;
            buffer.copy(data.data(), data.size());
            ID id = mask(PORT_TYPE::BLUETOOTH, id, this->m_group_id, this->m_port_id);
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
        // 清空所有特征和取消订阅
        LOGINFO("断开设备连接");
        for (auto&& [id, charac] : m_rxCharacteristics) {
            charac.ValueChanged(nullptr);
        }
        m_rxCharacteristics.clear();
        m_txCharacteristics.clear();

        if (m_device) {
            m_device.Close();
            m_device = nullptr;
        }
    }
};

} // namespace transport

#endif // ENABLE_WIN_BLUETOOTH
#endif // __WIN_BLE_PORT_HPP__