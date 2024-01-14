#ifndef __PACKAGE_MANAGER_HPP__
#define __PACKAGE_MANAGER_HPP__

#include <map>
#include <mutex>
#include <shared_mutex>
#include <opencv2/opencv.hpp>

#include "impls/BasePackage.hpp"
#include "impls/PackageInterface.hpp"
#include "utils/Utility.hpp"
#include "impls/logger.hpp"

namespace transport{

class PackageManager
{
private:
    std::unordered_map<ID, BasePackage::SharedPtr> m_package_map;
    std::shared_mutex m_package_map_mutex;
public:
    using SharedPtr = std::shared_ptr<PackageManager>;

    PackageManager() = default;

    PackageManager(std::string file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        if(fs.isOpened()) {
            for(auto port_node : fs["id_list"])
            {
                std::string port_name = port_node["port"];
                if(isCanPortName(port_name))
                {
                    for(auto can_id_info_node : port_node["id"])
                    {
                        int can_id_info = (int)can_id_info_node;
                        int id = can_id_info & 0xfff;
                        int flag = can_id_info >> 12;
                        add((CAN_ID)id, flag);
                    }
                }
                if(isSerialPortName(port_name))
                {
                    for(auto serial_id_info_node : port_node["id"])
                    {
                        int serial_id_info = (int)serial_id_info_node;
                        int id = serial_id_info & 0xfff;
                        int flag = serial_id_info >> 12;
                        add((SERIAL_ID)id, flag);
                    }
                }
            }
        }
        else {
            LOGERROR("Package Manager cannot open config file %s", file_path.c_str());
            throw PORT_EXCEPTION("Package Manager cannot open config file " + file_path);
        }
    }

    template<typename IDType>
    void add(IDType package_id, int flag = 0)
    {
        ID id = mask(package_id);
        std::lock_guard write_lock(m_package_map_mutex);
        // 已经有的id加不进去
        if(m_package_map.find(id) == m_package_map.end()) {
            auto debug_flag = flag & 0xff;
            auto queue_size = (flag >> 8) & 0xff;
            queue_size = queue_size > 1 ? queue_size : 1;

            BasePackage::SharedPtr package_ptr = std::make_shared<BasePackage>(id, debug_flag);
            m_package_map[id] = package_ptr;
        }
        else
        {
            LOGWARN("id 0x%x had already existed in PackageManager::m_package_map", (int)id);
        }
    }

    void add(BasePackage::SharedPtr package_ptr)
    {
        std::lock_guard write_lock(m_package_map_mutex);
        ID id = package_ptr->m_id;
        if(m_package_map.find(id) == m_package_map.end()) {
            m_package_map[id] = package_ptr;
        }
        else
        {
            LOGWARN("id 0x%x had already existed in PackageManager::m_package_map", (int)id);
        }
    }

    template<typename IDType>
    BasePackage::SharedPtr get(IDType package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id 0x%x in PackageManager::m_package_map, it will return nullptr", __PRETTY_FUNCTION__, (int)id);
        return nullptr;
    }

    template<typename IDType>
    BasePackage::SharedPtr operator[](IDType package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id 0x%x in PackageManager::m_package_map, it will return nullptr", __PRETTY_FUNCTION__, (int)id);
        return nullptr;
    }

    template<typename IDType>
    bool find(IDType package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return true;
        LOGWARN("%s: not found id 0x%x in PackageManager::m_package_map", __PRETTY_FUNCTION__, (int)id);
        return false;
    }

    template <typename T, typename IDType>
    void send(IDType package_id, T &package)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        auto package_ptr = m_package_map[id];
        read_lock.unlock();

        if(package_ptr == nullptr) {
            LOGERROR("in function %s :PackageManager::m_package_map does not contain id 0x%x, target type is %s. config里是不是没把这个包添加进去?", __PRETTY_FUNCTION__, (int)id, __TYPE(T));
            return;
        }

        Buffer buffer;
        buffer << package;

#ifdef __DEBUG__
        if(package_ptr->m_debug_flag & DEBUG_PRINT_BUFFER) {
            LOGDEBUG("[Debug Print]: buffer id 0x%x : %s", (int)id, buffer.toString().c_str());
        }
        if(package_ptr->m_debug_flag & DEBUG_PRINT_TARGET) {
            LOGDEBUG("[Debug Print]: buffer id 0x%x, package type %s : \n%s", (int)id, __TYPE(T), package.toString().c_str());
        }
#endif // __DEBUG__
        package_ptr->sendBuffer(buffer, id);
    }

    template <typename T, typename IDType>
    T recv(IDType package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        auto package_ptr = m_package_map[id];
        read_lock.unlock();
        if(package_ptr == nullptr) {
            LOGERROR("in function %s :PackageManager::m_package_map does not contain id 0x%x, target type is %s. config里是不是没把这个包添加进去?", __PRETTY_FUNCTION__, (int)id, __TYPE(T));
            return T();
        }

        Buffer buffer = package_ptr->readBuffer().buffer;
        if(buffer.empty()) {
            LOGWARN("in funcion %s :buffer is empty, id is 0x%x, target type is %s, maybe you have never received this package!", __PRETTY_FUNCTION__, (int)id, __TYPE(T));
        }

        T target;
        target << buffer;

#ifdef __DEBUG__
        if(package_ptr->m_debug_flag & DEBUG_PRINT_BUFFER) {
            LOGDEBUG("[Debug Print]: buffer id 0x%x : %s", (int)id, buffer.toString().c_str());
        }
        if(package_ptr->m_debug_flag & DEBUG_PRINT_TARGET) {
            LOGDEBUG("[Debug Print]: buffer id 0x%x, target type %s : \n%s", (int)id, __TYPE(T), target.toString().c_str());
        }
#endif // __DEBUG__
        return target;
    }

    // 从port收数据
    template <typename IDType>
    void recv(BufferWithTime &buffer, IDType package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        m_package_map[id]->recvBuffer(buffer);
    }
};

} // namespace transport

#endif // __PACKAGE_MANAGER_HPP__