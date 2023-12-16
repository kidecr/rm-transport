#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <vector>
#include <iostream>
#include <time.h>
#include <queue>
#include <memory>
#include <cstring>
#include <cxxabi.h>

#include <sys/time.h>
#include <sys/stat.h>
#include <sched.h>


#include "error.hpp"

#ifndef __CLASS__
// 用于类内，内容为类名，string类型
#define __CLASS__ (abi::__cxa_demangle(typeid(*this).name(), 0, 0, 0))
#endif // __CLASS__

#ifndef __TYPE
// 内容为参数type的名，string类型
#define __TYPE(type) (abi::__cxa_demangle(typeid(type).name(), 0, 0, 0))
#endif // __TYPE()

#ifndef PI
#define PI 3.14159265358979323846
#endif // PI

#ifndef TO_STR
#define TO_STR(var) #var << ": " << var
#endif // TO_STR

#ifndef PORT_ASSERT
#define PORT_ASSERT( expr ) do { if(!!(expr)) ; else throw transport::PortException(#expr, __PRETTY_FUNCTION__, __FILE__, __LINE__ ); } while(0)
#endif // PORT_ASSERT

#ifndef PORT_EXCEPTION
#define PORT_EXCEPTION( msg ) transport::PortException(( msg ), __PRETTY_FUNCTION__, __FILE__, __LINE__ )
#endif // PORT_EXCEPTION

// IENUM和UENUM是两个不会占用内存空间的常量类型，有作用域，用于解决ENUM没有作用域而class ENUM没发直接转int的问题
#ifndef IENUM
#define IENUM static constexpr int 
#endif // IENUM

#ifndef UENUM
#define UENUM static constexpr unsigned int 
#endif // UENUM

// 设置Buffer最大大小
#ifndef MAX_BUFFER_SIZE
#define MAX_BUFFER_SIZE 64
#endif // MAX_BUFFER_SIZE

/**************************************类**********************************************/

namespace transport{

/**
 * @brief 
 * @note 本身没有面向多线程的设计
 * 
 */
class Buffer
{
public:
    uint8_t data[MAX_BUFFER_SIZE];
    int length;

    Buffer(int size = 0) : data{0}, length(size)
    {}

    // // gcc编译器可以直接拷贝
    // // 这个函数不支持多线程，十分容易出问题，可能和memmove有关
    // Buffer(const Buffer& buffer)
    // {
    //     memmove(this->data, buffer.data, buffer.length);
    //     length = buffer.length;
    // }
    
    /**
     * @brief 将src拷贝到buffer
     * 
     * @param src 输入
     * @param size 输入大小
     * @return int 拷贝字节数
     */
    inline int copy(uint8_t* src, int size)
    {
        PORT_ASSERT(size <= MAX_BUFFER_SIZE && size > 0);
        memmove(this->data, src, size);
        length = size;
        return length;
    }

    /**
     * @brief 将buffer拷贝到dst
     * 
     * @param dst 输出
     * @param size 可输出大小，即输出buffer的长度
     * @return int 拷贝字节数
     */
    inline int copyTo(uint8_t* dst, int size)
    {
        PORT_ASSERT(size > length);
        memmove(dst, this->data, length);
        return length;
    }

    /**
     * @brief 在buffer中用下标索引
     * 
     * @param index 下标
     * @return uint8_t& 
     */
    inline uint8_t& operator [](int index)
    {
        PORT_ASSERT(index < MAX_BUFFER_SIZE && index >= 0);
        if(index >= length) throw PORT_EXCEPTION("index out of length");

        return data[index];
    }
#ifndef USE_LOCKFREE_QUEUE
    // gcc编译器可以直接拷贝类中的数组
    inline const Buffer& operator =(const Buffer& src)
    {
        memmove(this->data, src.data, src.length);
        length = src.length;
        return src;
    }
#endif // USE_LOCKFREE_QUEUE
    inline int size()
    {
        return length;
    }

    /**
     * @brief 同vector的resize
     * 
     * @param size 
     * @param n 
     * @return int 
     */
    inline int resize(size_t size, int n = 0)
    {
        PORT_ASSERT(size <= MAX_BUFFER_SIZE);
        if(size > length) {
            memset(this->data + length, n, size - length);
        }
        length = size;
        return length;
    }

    inline void clear()
    {
        length = 0;
    }

    inline bool empty()
    {
        return length == 0;
    }

    inline void push_back(uint8_t c)
    {
        if(length + 1 >= MAX_BUFFER_SIZE) throw PORT_EXCEPTION("buffer size out of range");
        data[length] = c;
        ++length;
    }

    std::string toString() 
    {
        std::stringstream ss;
        ss << "{";
        for(auto i = 0; i < length; ++i) {
            ss << "0x" << std::hex << (int)data[i] << ", ";
        }
        ss << "}";
        return ss.str();
    }

    ~Buffer() = default;

};

#ifdef USE_LOCKFREE_QUEUE
#include <boost/lockfree/queue.hpp>
#endif // USE_LOCKFREE_QUEUE

/**
 * @brief 类型定义
 * 
 */
// typedef std::vector<uint8_t> Buffer;
typedef std::queue<Buffer> BufferQueue;
typedef struct {Buffer buffer; timeval tv;} BufferWithTime;
typedef std::queue<BufferWithTime> BufferWithTimeQueue;
typedef std::shared_ptr<BufferWithTimeQueue> BufferWithTimeQueuePtr;
typedef struct { Buffer buffer; int id;} BufferWithID;
#ifndef USE_LOCKFREE_QUEUE
typedef std::queue<BufferWithID> BufferWithIDQueue;
#else
#if __cplusplus >= 202002L
// C++20中将rebind移除了，而我使用的boost库版本需要使用rebind，因此增加了定义
template<class T>
struct allocator_for_cxx20 : public std::allocator<T>
{
    template<class U>
    struct rebind{
        typedef allocator_for_cxx20<U> other;
    };
};
typedef boost::lockfree::queue<BufferWithID,
                               boost::lockfree::fixed_sized<true>, 
                               boost::lockfree::capacity<256>, 
                               boost::lockfree::allocator<allocator_for_cxx20<void>>
                               > BufferWithIDQueue;
#else
typedef boost::lockfree::queue<BufferWithID,
                               boost::lockfree::fixed_sized<true>, 
                               boost::lockfree::capacity<256>
                               > BufferWithIDQueue;
#endif // C++20
#endif // USE_LOCKFREE_QUEUE


/**
 * @brief 接口负载
 * 
 */
class Workload
{
private:
    time_t m_last_sec;
    int m_last_sec_count;
    int m_cur_count;
    bool m_update;

public:
    /**
     * @brief 更新一次接口负载，本质上就是负载加一
     * 
     */
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

    /**
     * @brief 获取上一秒负载都值
     * 
     * @return int 负载值
     */
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
        m_update = false;
        return m_last_sec_count;
    }

    friend std::ostream &operator<<(std::ostream &ostream, Workload &workload)
    {
        ostream << workload.m_last_sec_count;
        return ostream;
    }
};

/**
 * @brief 接口负载
 * 
 */
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
    IENUM Available = 1;
    IENUM Unavailable = 0;
    IENUM Deprecaped = -1;
public:
    std::string port_name;  // 接口名
    int status;             // 可用状态 1:可用, 0:不可用, -1:该口已经迁移完成
    int group;              // 接口所在组别
    PortWorkloads workload; // 接口工作负载

public:
    PortStatus(): status{Unavailable}, group{0} {}
};

} // namespace transport

/**************************************函数********************************************/

/**
 * @brief 设置线程内核绑定(亲和度)
 * 
 * @param cpu_id 目标内核
 */
inline void set_cpu_affinity(int cpu_id) {
    cpu_set_t set;
    CPU_ZERO(&set);
    // 设置绑定的核
    CPU_SET(cpu_id, &set);
    // 设置绑定当前进程
    sched_setaffinity(0, sizeof(cpu_set_t), &set);
}


/**
 * @brief 清空内存
 * @param t 目标指针
 */
template <typename T>
inline void clear(T *t)
{
    memset(t, 0, sizeof(T));
}

/**
 * @brief 获取当前时间
 * 
 * @return timeval 
 */
inline timeval gettimeval()
{
    timeval tv;
    gettimeofday(&tv, nullptr);
    return tv;
}

//@brief 用cout输出buffer， 16进制
std::ostream& operator <<(std::ostream &stream, transport::Buffer &buffer)
{
    stream << std::hex;
    for(size_t i = 0; i < buffer.size(); ++i)
        stream << (uint32_t)buffer[i] << " ";
    stream << std::dec;
    return stream;
}


//@brief 比较时间大小
static bool operator>(timeval t1, timeval t2)
{
    if (t1.tv_sec > t2.tv_sec)
        return true;
    else if (t1.tv_sec == t2.tv_sec && t1.tv_usec > t2.tv_usec)
        return true;
    else
        return false;
    return false;
}

//@brief 比较时间大小
static bool operator>=(timeval t1, timeval t2)
{
    if (t1.tv_sec > t2.tv_sec)
        return true;
    else if (t1.tv_sec == t2.tv_sec && t1.tv_usec >= t2.tv_usec)
        return true;
    else
        return false;
    return false;
}

//@brief 比较时间是否相等
static bool operator==(timeval t1, timeval t2)
{
    if (t1.tv_sec == t2.tv_sec && t1.tv_usec == t2.tv_usec)
        return true;
    else
        return false;
    return false;
}

//@brief 计算时间差
static timeval operator-(timeval t1, timeval t2)
{
    if(t1 >= t2) {
        timeval t3;
        t3.tv_sec = t1.tv_sec - t2.tv_sec;
        t3.tv_usec = t1.tv_usec - t2.tv_usec;
        return t3;
    }
    else {
        timeval t3 = t2 - t1;
        t3.tv_sec = 0 - t3.tv_sec;
        t3.tv_usec = 0 - t3.tv_usec;
        return t3;
    }
}
//@brief cout输出timeval
static std::ostream& operator <<(std::ostream &stream, timeval &tv)
{
    stream << "sec:" << tv.tv_sec << " usec:" << tv.tv_usec;
    return stream;
}

/**
 * @brief 判断路径是否是文件夹
 * 
 * @param directory 
 * @return true 
 * @return false 
 */
bool isFolder(const char* directory)
{
    if(access(directory, F_OK) != 0) // 路径文件不存在
        return false;
    struct stat file_stat;
    if(stat(directory, &file_stat) == 0 && S_ISDIR(file_stat.st_mode))
        return true;
    return false;
}

/**
 * @brief 判断路径是否是文件
 * 
 * @param directory 
 * @return true 
 * @return false 
 */
bool isFile(const char* directory)
{
    if(access(directory, F_OK) != 0) // 文件不存在
        return false;
    struct stat file_stat;
    if(stat(directory, &file_stat) == 0 && S_ISREG(file_stat.st_mode))
        return true;
    return false;
}

/**
 * @brief 创建多级目录
 * 
 * @param directory 目录
 */
void createDirectory(const char *directory)				//创建完整的多级目录
{
    char parent_directory[255] = {0};			//用于存储上级目录
    int len = strlen(directory);

    while(directory[len] != '/')	
    {
            len--;
    }

    strncpy(parent_directory, directory, len);		//存储上级目录

    if(access(parent_directory, F_OK) == 0)	    //若上级目录已存在
    {
            int i = 0;
            mkdir(directory, 0755);		    //创建目标目录
    }
    else
    {
            createDirectory(parent_directory);  //递归创建上级目录
            mkdir(directory, 0755);		    //创建上级目录后创建目标目录
    }
}


#endif // __UTILITY_HPP__