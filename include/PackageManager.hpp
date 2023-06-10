#ifndef __PACKAGE_MANAGER_HPP__
#define __PACKAGE_MANAGER_HPP__

#include <map>

#include <Package.hpp>
#include <WMJProtocol.h>

class PackageManager
{
    std::map<CAN_ID, BasePackage::SharedPtr> m_package_map;

public:
    using SharedPtr = std::shared_ptr<PackageManager>;

    void add(BasePackage::SharedPtr package_ptr)
    {
        CAN_ID id = package_ptr->m_can_id;
        m_package_map[id] = package_ptr;
    }

    BasePackage::SharedPtr get(CAN_ID id)
    {
        if(m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        return nullptr;
    }

    BasePackage::SharedPtr operator[](CAN_ID id)
    {
        if(m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        return nullptr;
    }

    bool have(CAN_ID id)
    {
        if(m_package_map.find(id) != m_package_map.end())
            return true;
        return false;
    }

    template<typename T>
    void send(CAN_ID id, T package)
    {
        auto package_ptr = m_package_map[id];
        typename PackageInterFace<T>::SharedPtr package_interface_ptr = std::static_pointer_cast<PackageInterFace<T>>(package_ptr);
        Buffer buffer = package_interface_ptr->encode(package);
        package_interface_ptr->sendBuffer(buffer, id);
    }

    template<typename T>
    T rvec(CAN_ID id)
    {
        auto package_ptr = m_package_map[id];
        // PackageInterFace<T>::SharedPtr package_interface_ptr = static_cast<PackageInterFace<T>::SharedPtr>(package_ptr);
        typename PackageInterFace<T>::SharedPtr package_interface_ptr = std::static_pointer_cast<PackageInterFace<T>>(package_ptr);
        // typename PackageInterFace<T>::template SharedPtr package_interface_ptr = std::dynamic_pointer_cast<typename PackageInterFace<T>>(package_ptr);
        Buffer buffer = package_interface_ptr->readBuffer().first;
        return package_interface_ptr->decode(buffer);
    }
};

#endif // __PACKAGE_MANAGER_HPP__