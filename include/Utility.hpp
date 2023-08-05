#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <vector>
#include <time.h>

#include <queue>
#include <memory>

#include <sys/time.h>
#include <sched.h>

#ifndef PORT_ASSERT
#define PORT_ASSERT( expr ) do { if(!!(expr)) ; else throw PortException(#expr, __func__, __FILE__, __LINE__ ); } while(0)
#endif // PORT_ASSERT

enum CAN_ID
{
    GIMBAL = 0x312,
    GIMBAL_GLOBAL = 0x316,
    GYRO = 0x314,
    SHOOT = 0x321,
    TIME = 0x345
};

typedef std::vector<uint8_t> Buffer;
typedef std::queue<Buffer> BufferQueue;
typedef std::pair<Buffer, int> BufferWithID;
typedef std::queue<BufferWithID> BufferWithIDQueue;
typedef std::pair<Buffer, timeval> BufferWithTime;
typedef std::queue<BufferWithTime> BufferWithTimeQueue;
typedef std::shared_ptr<BufferWithTimeQueue> BufferWithTimeQueuePtr;


class PortException : public std::exception
{
public:
    PortException(std::string message) { this->message = message; };
    PortException(std::string info, std::string func, std::string file, std::string line) {
        this->message = file + ":" + line + ": in " + func + " " + info; 
    }
    PortException(const char* info, const char* func, const char* file, int line) {
        this->message = std::string(file) + ":" + std::to_string(line) + ": in " + std::string(func) + " " + std::string(info); 
    }
    ~PortException(){};
    std::string message;
    const char* what()
    {
        if (message.empty())
            std::cout << "empty" << std::endl;
        return message.c_str();
    }
};


/**
 * @brief 设置线程内核绑定(亲和度)
 * 
 * @param cpu_id 目标内核
 */
void set_cpu_affinity(int cpu_id) {
    cpu_set_t set;
    CPU_ZERO(&set);
    // 设置绑定的核
    CPU_SET(cpu_id, &set);
    // 设置绑定当前进程
    sched_setaffinity(0, sizeof(cpu_set_t), &set);
}



template <typename T>
inline void clear(T *t)
{
    memset(t, 0, sizeof(T));
}



// 接口负载
class Workload
{
private:
    time_t m_last_sec;
    int m_last_sec_count;
    int m_cur_count;
    bool m_update;

public:
    void update()
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        if (m_last_sec == tv.tv_sec)
            ++m_cur_count;
        else
        {
            m_update = true;
            m_last_sec = tv.tv_sec;
            m_last_sec_count = m_cur_count;
            m_cur_count = 0;
        }
    }

    int getWorkload()
    {
        m_update = false;
        return m_last_sec_count;
    }

    bool canUpload()
    {
        return m_update;
    }

    int operator=(int workload)
    {
        m_last_sec_count = workload;
        return workload;
    }

    int operator+(Workload &workload)
    {
        return m_last_sec_count + workload.m_last_sec_count;
    }

    operator int()
    {
        return m_last_sec_count;
    }

    friend std::ostream &operator<<(std::ostream &ostream, Workload &workload)
    {
        ostream << workload.m_last_sec_count;
        return ostream;
    }
};

class PortWorkloads
{
public:
    Workload read;
    Workload write;

public:
    operator int()
    {
        int read_workload = read.getWorkload();
        int write_workload = write.getWorkload();
        return read_workload + write_workload;
    }

    int operator=(int workload)
    {
        read = workload;
        write = 0;
        return workload;
    }

    bool operator<(PortWorkloads &workload)
    {
        int A_workload = read + write;
        int B_workload = workload.read + workload.write;
        return A_workload < B_workload;
    }

    bool operator>(PortWorkloads &workload)
    {
        int A_workload = read + write;
        int B_workload = workload.read + workload.write;
        return A_workload > B_workload;
    }
};

// 接口信息表
class PortStatus
{
public:
    using SharedPtr = std::shared_ptr<PortStatus>;
    // 这么写不会占用空间
    constexpr static int Available = 1;
    constexpr static int Unavailable = 0;
    constexpr static int Deprecaped = -1;
public:
    std::string port_name;  // 接口名
    int status;             // 可用状态 1:可用, 0:不可用, -1:该口已经迁移完成
    int group;              // 接口所在组别
    PortWorkloads workload; // 接口工作负载

public:
    PortStatus(): status{Unavailable}, group{0} {}
};

std::ostream& operator <<(std::ostream &stream, Buffer &buffer)
{
    stream << std::hex;
    for(size_t i = 0; i < buffer.size(); ++i)
        stream << (uint32_t)buffer[i] << " ";
    stream << std::dec;
    return stream;
}


struct PortIDTable
{
    using SharedPtr = std::shared_ptr<PortIDTable>;
    std::string port_name;
    std::vector<CAN_ID> id_list;
};

#endif // __UTILITY_HPP__