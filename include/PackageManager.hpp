#ifndef __PACKAGE_MANAGER_HPP__
#define __PACKAGE_MANAGER_HPP__

#include <map>
#include <opencv2/opencv.hpp>

#include <Package.hpp>
#include <Utility.hpp>

class PackageManager
{
    std::unordered_map<CAN_ID, BasePackage::SharedPtr> m_package_map;

    // std::vector<PortIDTable> m_port_id_table;
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
        Buffer buffer;
        
        buffer << package;
        package_ptr->sendBuffer(buffer, id);
    }

    template <typename T>
    T recv(CAN_ID id)
    {
        auto package_ptr = m_package_map[id];
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