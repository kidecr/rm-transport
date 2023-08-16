#ifndef __PACKAGE_MANAGER_HPP__
#define __PACKAGE_MANAGER_HPP__

#include <map>
#include <shared_mutex>
#include <opencv2/opencv.hpp>

#include <Package.hpp>
#include <Utility.hpp>

class PackageManager
{
private:
    std::unordered_map<CAN_ID, BasePackage::SharedPtr> m_package_map;
    std::shared_mutex m_package_map_mutex;
public:
    using SharedPtr = std::shared_ptr<PackageManager>;

    PackageManager() = default;

    PackageManager(std::string file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        if(fs.isOpened()) {
            for(auto can : fs["id_list"])
            {
                std::string port_name = can["port"];
                for(auto can_id : can["id"])
                {
                    int id = can_id;
                    add((CAN_ID)id);
                }
            }
        }
        else {
            throw PortException("Port controller cannot open config file!");
        }
    }

    void add(CAN_ID id)
    {
        std::lock_guard write_lock(m_package_map_mutex);
        // 已经有的id加不进去
        if(m_package_map.find(id) == m_package_map.end()) {
            BasePackage::SharedPtr package_ptr = std::make_shared<BasePackage>(id);
            m_package_map[id] = package_ptr;
        }
    }

    void add(BasePackage::SharedPtr package_ptr)
    {
        std::lock_guard write_lock(m_package_map_mutex);
        CAN_ID id = package_ptr->m_can_id;
        if(m_package_map.find(id) == m_package_map.end()) {
            m_package_map[id] = package_ptr;
        }
    }

    BasePackage::SharedPtr get(CAN_ID id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        return nullptr;
    }

    BasePackage::SharedPtr operator[](CAN_ID id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        return nullptr;
    }

    bool find(CAN_ID id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return true;
        return false;
    }

    template <typename T>
    void send(CAN_ID id, T package)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        auto package_ptr = m_package_map[id];
        read_lock.unlock();

        Buffer buffer;
        buffer << package;
        package_ptr->sendBuffer(buffer, id);
    }

    template <typename T>
    T recv(CAN_ID id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        auto package_ptr = m_package_map[id];
        read_lock.unlock();

        Buffer buffer = package_ptr->readBuffer().buffer;
        T target;
        target << buffer;
        return target;
    }

    // 从port收数据
    void recv(BufferWithTime &buffer, CAN_ID can_id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        m_package_map[can_id]->recvBuffer(buffer);
    }
};

#endif // __PACKAGE_MANAGER_HPP__