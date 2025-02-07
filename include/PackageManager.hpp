#ifndef __PACKAGE_MANAGER_HPP__
#define __PACKAGE_MANAGER_HPP__

#include <utility>
#include <map>
#include <mutex>
#include <shared_mutex>

#include "utils/mask.hpp"
#include "utils/Utility.hpp"
#include "impls/BasePackage.hpp"
#include "impls/PackageInterface.hpp"
#include "impls/logger.hpp"
#include "impls/Config.hpp"
#include "impls/PackageID.hpp"

namespace transport{

struct UseTimestamp { explicit UseTimestamp() = default; };
constexpr UseTimestamp use_timestamp {};

class PackageManager
{
private:
    std::unordered_map<ID, BasePackage::SharedPtr> m_package_map;
    std::shared_mutex m_package_map_mutex;
public:
    using SharedPtr = std::shared_ptr<PackageManager>;

    /**
     * @brief 构造函数，负责根据config信息注册包id
     * 
     * @param config config
     */
    PackageManager(config::Config::SharedPtr config)
    {
        for(auto &package_info : config->m_package_list)
        {
            add(package_info.m_id, package_info.m_debug_flag, package_info.m_queue_size);
        }
    }

    /**
     * @brief 根据参数创建并注册一个新包
     * 
     * @tparam T 
     * @param package_id id
     * @param flag debug_flag
     * @param queue_size 收包队列大小（yaml配置文件时无法指定）
     */
    template<IDType T>
    void add(T package_id, int64_t flag = 0, int64_t queue_size = 1)
    {
        ID id = mask(package_id);
        std::lock_guard write_lock(m_package_map_mutex);
        // 已经有的id加不进去
        if(m_package_map.find(id) == m_package_map.end()) {
            BasePackage::SharedPtr package_ptr = std::make_shared<BasePackage>(id, flag, queue_size);
            m_package_map[id] = package_ptr;
        }
        else
        {
            LOGWARN("id 0x%lx had already existed in PackageManager::m_package_map", id);
        }
    }

    /**
     * @brief 注册一个新的包
     * 
     * @param package_ptr 
     */
    void add(BasePackage::SharedPtr package_ptr)
    {
        std::lock_guard write_lock(m_package_map_mutex);
        ID id = package_ptr->m_id;
        if(m_package_map.find(id) == m_package_map.end()) {
            m_package_map[id] = package_ptr;
        }
        else
        {
            LOGWARN("id 0x%lx had already existed in PackageManager::m_package_map", id);
        }
    }

    /**
     * @brief 根据id获取一个包指针
     * 
     * @tparam T 
     * @param package_id 
     * @return BasePackage::SharedPtr 当不存在时返回空指针
     */
    template<IDType T>
    BasePackage::SharedPtr get(T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id 0x%lx in PackageManager::m_package_map, it will return nullptr", __PRETTY_FUNCTION__, id);
        return nullptr;
    }

    /**
     * @brief 使用下标索引的方式获取一个包指针
     * 
     * @tparam T 包id类型
     * @param package_id 包id
     * @return BasePackage::SharedPtr 当不存在时返回空指针
     */
    template<IDType T>
    BasePackage::SharedPtr operator[](T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        LOGWARN("%s: no id 0x%lx in PackageManager::m_package_map, it will return nullptr", __PRETTY_FUNCTION__, id);
        return nullptr;
    }

    /**
     * @brief 判断指定id的包是否存在
     * 
     * @tparam T 包id类型
     * @param package_id 包id
     * @return true 指定id的包在接受或发送列表中存在
     * @return false 指定id的包在接受或发送列表中不存在
     */
    template<IDType T>
    bool find(T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        if (m_package_map.find(id) != m_package_map.end())
            return true;
        LOGWARN("%s: not found id 0x%lx in PackageManager::m_package_map", __PRETTY_FUNCTION__, id);
        return false;
    }
    /**
     * @brief 发包函数
     * 
     * @tparam T 发送包类型
     * @tparam T2 ID类型
     * @param package_id 包ID
     * @param package 待发送包
     */
    template <typename T, IDType T2>
    void send(T2 package_id, T &package)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        auto package_ptr = m_package_map[id];
        read_lock.unlock();

        if(package_ptr == nullptr) {
            LOGERROR("in function %s :PackageManager::m_package_map does not contain id 0x%lx, target type is %s. config里是不是没把这个包添加进去?", __PRETTY_FUNCTION__, id, __TYPE(T));
            return;
        }

        Buffer buffer;
        buffer << package;

#ifdef __DEBUG__
        if(package_ptr->m_debug_flag & DEBUG_PRINT_BUFFER) {
            LOGDEBUG("[Debug Print]: send buffer id 0x%lx : %s", id, buffer.toString().c_str());
        }
        if(package_ptr->m_debug_flag & DEBUG_PRINT_TARGET) {
            LOGDEBUG("[Debug Print]: send buffer id 0x%lx, package type %s : %s", id, __TYPE(T), package.toString().c_str());
        }
#endif // __DEBUG__
        package_ptr->sendBuffer(buffer, id);
    }
    /**
     * @brief 收包接口，返回解码后的Package
     * 
     * @tparam T Package类型
     * @tparam T2 ID类型
     * @param package_id 包ID
     * @return T 解码后的Package
     */
    template <typename T, IDType T2>
    T recv(T2 package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        auto package_ptr = m_package_map[id];
        read_lock.unlock();
        if(package_ptr == nullptr) {
            LOGERROR("in function %s :PackageManager::m_package_map does not contain id 0x%lx, target type is %s. config里是不是没把这个包添加进去?", __PRETTY_FUNCTION__, id, __TYPE(T));
            return T();
        }

        Buffer buffer = package_ptr->readBuffer().buffer;
        if(buffer.empty()) {
            LOGWARN("in funcion %s :buffer is empty, id is 0x%lx, target type is %s, maybe you have never received this package!", __PRETTY_FUNCTION__, id, __TYPE(T));
        }

        T target;
        target << buffer;

#ifdef __DEBUG__
        if(package_ptr->m_debug_flag & DEBUG_PRINT_BUFFER) {
            LOGDEBUG("[Debug Print]: recv buffer id 0x%lx : %s", id, buffer.toString().c_str());
        }
        if(package_ptr->m_debug_flag & DEBUG_PRINT_TARGET) {
            LOGDEBUG("[Debug Print]: recv buffer id 0x%lx, target type %s : %s", id, __TYPE(T), target.toString().c_str());
        }
#endif // __DEBUG__
        return target;
    }
    /**
     * @brief 收包接口，会在返回解码后的Package的同时返回收包时间戳
     * 
     * @tparam T Package类型
     * @tparam T2 ID类型
     * @param package_id 包ID
     * @param use_timestamp 直接传入use_timestamp，该变量为一个全局变量，从而显示指定重载该函数
     * @return std::pair<T, timeval> 返回一个pair，其中为解码后的Package和时间戳
     */
    template <typename T, IDType T2>
    std::tuple<T, timeval> recv(T2 package_id, UseTimestamp)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        auto package_ptr = m_package_map[id];
        read_lock.unlock();
        if(package_ptr == nullptr) {
            LOGERROR("in function %s :PackageManager::m_package_map does not contain id 0x%lx, target type is %s. config里是不是没把这个包添加进去?", __PRETTY_FUNCTION__, id, __TYPE(T));
            return std::make_tuple(T(), timeval());
        }
        BufferWithTime buffer_with_time = package_ptr->readBuffer();
        if(buffer_with_time.buffer.empty()) {
            LOGWARN("in funcion %s :buffer is empty, id is 0x%lx, target type is %s, maybe you have never received this package!", __PRETTY_FUNCTION__, id, __TYPE(T));
        }

        T target;
        target << buffer_with_time.buffer;

#ifdef __DEBUG__
        if(package_ptr->m_debug_flag & DEBUG_PRINT_BUFFER) {
            LOGDEBUG("[Debug Print]: recv buffer id 0x%lx : %s", id, buffer_with_time.buffer.toString().c_str());
        }
        if(package_ptr->m_debug_flag & DEBUG_PRINT_TARGET) {
            LOGDEBUG("[Debug Print]: recv buffer id 0x%lx, target type %s : %s", id, __TYPE(T), target.toString().c_str());
        }
#endif // __DEBUG__
        return std::make_tuple(target, buffer_with_time.tv);
    }

    /**
     * @brief 从Port接收数据
     * 
     * @tparam T id
     * @param buffer 带有接受时间戳的buffer数据
     * @param package_id 该buffer对应的包id
     */
    template <IDType T>
    void recv(BufferWithTime &buffer, T package_id)
    {
        ID id = mask(package_id);
        std::shared_lock read_lock(m_package_map_mutex);
        m_package_map[id]->recvBuffer(buffer);
    }

    /**
     * @brief 列出所有注册过的包id
     * 
     * @return std::string 格式化字符串
     */
    std::string toString()
    {
        std::stringstream ss;
        std::shared_lock read_lock(m_package_map_mutex);
        ss << "{" << __CLASS__ << " id_list:[" << std::hex;
        for (auto it : m_package_map)
        {
            ss << " 0x" << std::setw(sizeof(ID) * 2) << std::setfill('0') << it.first;
        }
        ss << "]}";
        return ss.str();
    }
};

} // namespace transport

#endif // __PACKAGE_MANAGER_HPP__