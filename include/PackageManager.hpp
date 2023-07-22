#ifndef __PACKAGE_MANAGER_HPP__
#define __PACKAGE_MANAGER_HPP__

#include <map>
#include <opencv2/opencv.hpp>

#include <Package.hpp>
#include <WMJProtocol.h>

struct PortIDTable
{
    using SharedPtr = std::shared_ptr<PortIDTable>;
    std::string port_name;
    std::vector<CAN_ID> id_list;
};

class PackageManager
{
    std::unordered_map<CAN_ID, BasePackage::SharedPtr> m_package_map;

    std::vector<PortIDTable> m_port_id_table;
public:
    using SharedPtr = std::shared_ptr<PackageManager>;

    void add(CAN_ID id)
    {
        // 已经有的id加不进去
        if(m_package_map.find(id) == m_package_map.end()) {
            BasePackage::SharedPtr package_ptr = std::make_shared<BasePackage>(id);
            m_package_map[id] = package_ptr;
        }
    }

    void add(BasePackage::SharedPtr package_ptr)
    {
        CAN_ID id = package_ptr->m_can_id;
        if(m_package_map.find(id) == m_package_map.end()) {
            m_package_map[id] = package_ptr;
        }
    }

    void addIDFromConfigFile(std::string file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        for(auto can : fs["can"])
        {
            std::string port_name = can["port"];
            for(auto can_id : can["id"])
            {
                int id = can_id;
                add((CAN_ID)id);
                bind((CAN_ID)id, port_name);
            }
        }
    }

    void bind(CAN_ID id, std::string port_name)
    {
        auto port_id_table = m_port_id_table.begin();
        for (; port_id_table != m_port_id_table.end(); ++port_id_table)
        {
            if(port_id_table->port_name == port_name)   // 存在port_name
            {
                size_t i = 0;
                for(; i < port_id_table->id_list.size(); ++i)
                {
                    if(id == port_id_table->id_list[i]) // id已存在,不操作
                        break;
                }
                if (i == 0 || i == port_id_table->id_list.size())   // id不存在，添加id
                {
                    port_id_table->id_list.push_back(id);
                }
            }
        }
        if(port_id_table == m_port_id_table.end())  // 不存在port_name，新建table
        {
            PortIDTable port_id_table;
            port_id_table.port_name = port_name;
            port_id_table.id_list.push_back(id);
            m_port_id_table.push_back(port_id_table);
        }
    }

    PortIDTable& getPortIDTable(std::string port_name)
    {
        for(auto &it : m_port_id_table)
        {
            if(it.port_name == port_name)
            return it;
        }
        PortIDTable port_id_table;
        port_id_table.port_name = port_name;
        m_port_id_table.push_back(port_id_table);
        return m_port_id_table[m_port_id_table.size() - 1];
    }

    BasePackage::SharedPtr get(CAN_ID id)
    {
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        return nullptr;
    }

    BasePackage::SharedPtr operator[](CAN_ID id)
    {
        if (m_package_map.find(id) != m_package_map.end())
            return m_package_map[id];
        return nullptr;
    }

    bool find(CAN_ID id)
    {
        if (m_package_map.find(id) != m_package_map.end())
            return true;
        return false;
    }

    template <typename T>
    void send(CAN_ID id, T package)
    {
        auto package_ptr = m_package_map[id];
        // typename PackageInterFace<T>::SharedPtr package_interface_ptr = std::static_pointer_cast<PackageInterFace<T>>(package_ptr);
        Buffer buffer;
        buffer << package;
        package_ptr->sendBuffer(buffer, id);
    }

    template <typename T>
    T recv(CAN_ID id)
    {
        auto package_ptr = m_package_map[id];
        // PackageInterFace<T>::SharedPtr package_interface_ptr = static_cast<PackageInterFace<T>::SharedPtr>(package_ptr);
        // typename PackageInterFace<T>::SharedPtr package_interface_ptr = std::static_pointer_cast<PackageInterFace<T>>(package_ptr);
        // typename PackageInterFace<T>::template SharedPtr package_interface_ptr = std::dynamic_pointer_cast<typename PackageInterFace<T>>(package_ptr);
        Buffer buffer = package_ptr->readBuffer().first;
        T target;
        target << buffer;
        return target;
    }

    // 从port收数据
    void recv(BufferWithTime &buffer, CAN_ID can_id)
    {
        m_package_map[can_id]->recvBuffer(buffer);
    }
};

#endif // __PACKAGE_MANAGER_HPP__