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
                        int id = get_package_id(can_id_info);
                        int flag = get_id_flag(can_id_info);
                        add((CAN_ID)id, flag);
                    }
                }
                if(isSerialPortName(port_name))
                {
                    for(auto serial_id_info_node : port_node["id"])
                    {
                        int serial_id_info = (int)serial_id_info_node;
                        int id = get_package_id(serial_id_info);
                        int flag = get_id_flag(serial_id_info);
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

    template<IDType T>
    void add(T package_id, int flag = 0)
    {
        ID id = mask(package_id);
        std::lock_guard write_lock(m_package_map_mutex);
        // 已经有的id加不进去
        if(m_package_map.find(id) == m_package_map.end()) {
            int debug_flag = get_debug_flag(flag);
            int queue_size = get_queue_size(flag);

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

    template<IDType T>
    BasePackage::SharedPtr get(T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id 0x%x in PackageManager::m_package_map, it will return nullptr", __PRETTY_FUNCTION__, (int)id);
        return nullptr;
    }

    template<IDType T>
    BasePackage::SharedPtr operator[](T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id 0x%x in PackageManager::m_package_map, it will return nullptr", __PRETTY_FUNCTION__, (int)id);
        return nullptr;
    }

    template<IDType T>
    bool find(T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return true;
        LOGWARN("%s: not found id 0x%x in PackageManager::m_package_map", __PRETTY_FUNCTION__, (int)id);
        return false;
    }

    template <typename T, IDType T2>
    void send(T2 package_id, T &package)
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

    template <typename T, IDType T2>
    T recv(T2 package_id)
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
    template <IDType T>
    void recv(BufferWithTime &buffer, T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        m_package_map[id]->recvBuffer(buffer);
    }
};

} // namespace transport

#endif // __PACKAGE_MANAGER_HPP__