#ifndef __PACKAGE_MANAGER_HPP__
#define __PACKAGE_MANAGER_HPP__

#include <map>
#include <shared_mutex>
#include <opencv2/opencv.hpp>

#include "BasePackage.hpp"
#include "PackageInterface.hpp"
#include "Utility.hpp"
#include "logger.hpp"

namespace transport{

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
            LOGERROR("Package Manager cannot open config file %s", file_path.c_str());
            throw PORT_EXCEPTION("Package Manager cannot open config file " + file_path);
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
        else
        {
            LOGWARN("id %x had already existed in PackageManager::m_package_map", (int)id);
        }
    }

    void add(BasePackage::SharedPtr package_ptr)
    {
        std::lock_guard write_lock(m_package_map_mutex);
        CAN_ID id = package_ptr->m_can_id;
        if(m_package_map.find(id) == m_package_map.end()) {
            m_package_map[id] = package_ptr;
        }
        else
        {
            LOGWARN("id %x had already existed in PackageManager::m_package_map", (int)id);
        }
    }

    BasePackage::SharedPtr get(CAN_ID id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id %x in PackageManager::m_package_map", __PRETTY_FUNCTION__, (int)id);
        return nullptr;
    }

    BasePackage::SharedPtr operator[](CAN_ID id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id %x in PackageManager::m_package_map", __PRETTY_FUNCTION__, (int)id);
        return nullptr;
    }

    bool find(CAN_ID id)
    {
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return true;
        LOGWARN("%s: not found id %x in PackageManager::m_package_map", __PRETTY_FUNCTION__, (int)id);
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
        if(package_ptr == nullptr)
            LOGWARN("package ptr is empty, id is %x, type is %s", (int)id, __TYPE(T));

        Buffer buffer = package_ptr->readBuffer().buffer;
        if(buffer.empty())
            LOGWARN("buffer is empty, target type is %s", __TYPE(T));
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

} // namespace transport

#endif // __PACKAGE_MANAGER_HPP__