#ifndef __WIN_BLE_PORT_HPP__
#define __WIN_BLE_PORT_HPP__

#define ENABLE_WIN_BLUETOOTH
#ifdef ENABLE_WIN_BLUETOOTH

#include <array>
#include <atomic>
#include <regex>
#include <future>

#include "impls/Port.hpp"

#include <winrt/windows.foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Devices.Enumeration.h>
#include <winrt/windows.devices.bluetooth.advertisement.h>

using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace winrt::Windows::Devices::Bluetooth::Advertisement;
using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Storage;
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
        LOGDEBUG("WinBLEPort::~WinBLEPort() destroyed.");
    }

    bool reinit() override {
        // std::lock_guard lock(m_deviceMutex);
        m_port_is_available = false;
        TryConnectDevice(true); // 强制重新连接
        return m_port_is_available;
    }

private:
    // BLE特征标识
    winrt::hstring m_targetDeviceName;

    // BLE连接状态
    BluetoothLEDevice m_device{nullptr};
    std::unordered_map<uint16_t, GattCharacteristic> m_txCharacteristics;
    std::unordered_map<uint16_t, GattCharacteristic> m_rxCharacteristics;
    std::unordered_map<uint16_t, winrt::guid> m_short2UUID;

    // 线程管理
    std::jthread m_workerThread;
    std::atomic_bool m_stopRequested{false};
    std::mutex m_deviceMutex;
    std::atomic_int m_retryCount{0};

    // UUID转换工具
    static uint16_t ExtractShortUUID(const winrt::guid& uuid) {
        // 检查标准蓝牙UUID格式：0000xxxx-{0000-1000-8000-00805f9b34fb}
        const uint8_t baseUUID[] = {0x00,0x00,0x10,0x00,0x80,0x00,0x00,0x80,0x5F,0x9B,0x34,0xFB};
        if(memcmp(uuid.Data4, baseUUID, sizeof(baseUUID)) == 0 &&
           uuid.Data3 == 0x0000 &&
           (uuid.Data1 & 0xFFFF0000) == 0x00000000) {
            return static_cast<uint16_t>(uuid.Data1 & 0xFFFF);
        }
        return 0; // 非标准短UUID
    }

    static std::string GuidToUuidString(const winrt::guid& guid) {
        char uuidStr[37]; // 32 characters for the GUID, 4 for '-', and 1 for null terminator

        snprintf(uuidStr, sizeof(uuidStr),
            "%08lX-%04hX-%04hX-%02X%02X-%02X%02X%02X%02X%02X%02X",
            guid.Data1, guid.Data2, guid.Data3,
            guid.Data4[0], guid.Data4[1], guid.Data4[2],
            guid.Data4[3], guid.Data4[4], guid.Data4[5],
            guid.Data4[6], guid.Data4[7]);
        return std::string(uuidStr);
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

    void TryConnectDevice(bool forceReconnect = false) {
        UpdatePortStatus(PortStatus::Connecting);
        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            if (ConnectDevice(forceReconnect)) {
                UpdatePortStatus(PortStatus::Available);
                return;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(RECONNECT_INTERVAL_MS));
        }
        LOGERROR("Failed to connect after %d attempts", MAX_RETRIES);
        UpdatePortStatus(PortStatus::Unavailable);
    }

    bool ConnectDevice(bool forceReconnect = false) {
        try {
            auto device = FindBLEDevice(m_targetDeviceName).get();
            if (!device) return false;

            std::lock_guard lock(m_deviceMutex);
            if (forceReconnect) {
                LOGDEBUG("Force reconnecting to device");
                DisconnectDevice();
            }

            return InitializeServices(device).get();
        } catch (const winrt::hresult_error& e) {
            LOGERROR("BLE连接失败: 0x%X %ls", 
                    e.code(), e.message().c_str());
        }
        return false;
    }

    IAsyncOperation<BluetoothLEDevice> FindBLEDevice(winrt::hstring targetDeviceName) {
        // 创建 BLE 广播监听器
        BluetoothLEAdvertisementWatcher watcher;
        watcher.ScanningMode(BluetoothLEScanningMode::Active);

        // 创建一个任务完成事件
        bool completion_event = false;
        BluetoothLEDevice result_device = nullptr;

        // 监听广播事件
        auto token = watcher.Received([&completion_event, &result_device, targetDeviceName](BluetoothLEAdvertisementWatcher w, BluetoothLEAdvertisementReceivedEventArgs e) {
            if (e.AdvertisementType() == BluetoothLEAdvertisementType::ConnectableUndirected) {
                auto name = e.Advertisement().LocalName();
                if (name == targetDeviceName) {
                    // 找到目标设备
                    try {
                        w.Stop(); // 停止监听
                        result_device = BluetoothLEDevice::FromBluetoothAddressAsync(e.BluetoothAddress()).get();
                        completion_event = true; // 设置任务完成源的结果
                    }
                    catch (...) {
                        //promise->set_exception(std::current_exception()); // 处理异常
                        w.Stop();
                    }
                }
            }
            });

        // 启动监听
        watcher.Start();

        while (!completion_event) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 返回异步操作
        co_return result_device;
    }

    IAsyncOperation<bool> InitializeServices(BluetoothLEDevice device) {
        co_await winrt::resume_background();
        
        try {
            // 清空特征映射
            m_txCharacteristics.clear();
            m_rxCharacteristics.clear();

            // 获取所有服务
            auto&& servicesResult = co_await device.GetGattServicesAsync();
            if (servicesResult.Status() != GattCommunicationStatus::Success) {
                LOGERROR("获取服务失败: %d", (int)servicesResult.Status());
                co_return false;
            }

            // 遍历所有服务
            for (auto&& service : servicesResult.Services()) {
                auto&& charsResult = co_await service.GetCharacteristicsAsync();
                if (charsResult.Status() != GattCommunicationStatus::Success) {
                    continue;
                }
                LOGDEBUG("Get Service: %s", GuidToUuidString(service.Uuid()).c_str());

                // 处理特征
                for (auto&& characteristic : charsResult.Characteristics()) {
                    auto properties = characteristic.CharacteristicProperties();
                    auto uuid = characteristic.Uuid();
                    auto shortid = BluetoothUuidHelper::TryGetShortId(uuid); 
                    uint16_t shortUUID = 0;
                    if (!shortid) {
                        shortUUID = uuid.Data1 & 0xFFFF;
                        m_short2UUID.emplace(shortUUID, uuid);
                        LOGDEBUG("Get Long UUID, %ls, ShortUUID: %04X", GuidToUuidString(uuid).c_str(), shortUUID);
                    }
                    else {
                        shortUUID = shortid.Value();
                        m_short2UUID.emplace(shortUUID, uuid);
                        LOGDEBUG("Get Short UUID, ShortUUID: %04X", shortUUID);
                    }

                    // 处理发送特征
                    if ((properties & GattCharacteristicProperties::Write) != GattCharacteristicProperties::None || 
                        (properties & GattCharacteristicProperties::WriteWithoutResponse) != GattCharacteristicProperties::None) {
                        m_txCharacteristics.emplace(shortUUID, characteristic);
                        LOGDEBUG("Found TX characteristic: %04X", shortUUID);
                    }

                    // 处理接收特征
                    if ((properties & GattCharacteristicProperties::Notify) != GattCharacteristicProperties::None) {
                        // 订阅通知
                        auto status = co_await characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                            GattClientCharacteristicConfigurationDescriptorValue::Notify);
                        
                        if (status == GattCommunicationStatus::Success) {
                            LOGDEBUG("Subscribed to notifications for characteristic: %04X", shortUUID);
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
            LOGERROR("捕获异常，初始化服务失败");
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
            Streams::DataWriter writer;
            writer.WriteBytes(winrt::array_view(data.data(), data.data() + data.size()));
            auto result = it->second.WriteValueAsync(writer.DetachBuffer()).get();
            return result == GattCommunicationStatus::Success;
        } catch (...) {
            return false;
        }
    }

    void OnCharacteristicChanged(uint16_t short_id, 
                               GattValueChangedEventArgs const& args) {
        try {
            Streams::DataReader reader = Streams::DataReader::FromBuffer(args.CharacteristicValue());
            auto len = reader.UnconsumedBufferLength();
            std::vector<uint8_t> data(len);
            reader.ReadBytes(data);

            Buffer buffer;
            buffer.copy(data.data(), data.size());
            ID id = mask(PORT_TYPE::BLUETOOTH, short_id, this->m_group_id, this->m_port_id);
            recvOnePackage(id, buffer);
        } catch (...) {
            LOGERROR("数据处理异常");
        }
    }

    void UpdatePortStatus(auto connect_status) {
        //std::lock_guard lock(m_deviceMutex);
        m_port_is_available = (connect_status == PortStatus::Available);
        if (m_port_status) {
            m_port_status->status = connect_status;
        }
    }

    void DisconnectDevice() {
        // 清空所有特征和取消订阅
        LOGINFO("disconnect called, 断开设备连接");
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