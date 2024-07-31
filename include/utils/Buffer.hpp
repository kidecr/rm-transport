#ifndef __BUFFER_HPP__
#define __BUFFER_HPP__

#include <queue>
#include <memory>
#include <string>
#include <sstream>
#include <cstring>

#include "impls/exception.hpp"
#include "protocal/Protocal.hpp"
#include "impls/PackageID.hpp"

#ifdef USE_LOCKFREE_QUEUE
#include <boost/lockfree/queue.hpp>
#endif // USE_LOCKFREE_QUEUE

// 设置Buffer最大大小
#ifndef MAX_BUFFER_SIZE
#define MAX_BUFFER_SIZE 128
#endif // MAX_BUFFER_SIZE

namespace transport{

#ifndef USE_STD_VECTOR

/**
 * @brief 
 * @note 本身没有面向多线程的设计
 * 
 */
class Buffer
{
    uint8_t m_data[MAX_BUFFER_SIZE];
    size_t m_length;

public:

    Buffer(size_t size = 0) : m_data{0}, m_length(size)
    {}

    // // gcc编译器可以直接拷贝
    // // 这个函数不支持多线程，十分容易出问题，可能和memmove有关
    // Buffer(const Buffer& buffer)
    // {
    //     memmove(this->m_data, buffer.m_data, buffer.m_length);
    //     m_length = buffer.m_length;
    // }
    
    /**
     * @brief 将src拷贝到buffer
     * 
     * @param src 输入
     * @param size 输入大小
     * @return size_t 拷贝字节数
     */
    inline size_t copy(uint8_t* src, size_t size)
    {
        PORT_ASSERT(size <= MAX_BUFFER_SIZE && size > 0);
        memmove(this->m_data, src, size);
        m_length = size;
        return m_length;
    }

    /**
     * @brief 将buffer拷贝到dst
     * 
     * @param dst 输出
     * @param size 可输出大小，即输出buffer的长度
     * @return size_t 拷贝字节数
     */
    inline size_t copyTo(uint8_t* dst, size_t size)
    {
        PORT_ASSERT(size <= m_length);
        memmove(dst, this->m_data, m_length);
        return m_length;
    }

    /**
     * @brief 在buffer中用下标索引
     * 
     * @param index 下标
     * @return uint8_t& 
     */
    inline uint8_t& operator [](size_t index)
    {
        PORT_ASSERT(index < MAX_BUFFER_SIZE && index >= 0);
        if(index >= m_length) throw PORT_EXCEPTION("index out of m_length");

        return m_data[index];
    }
#ifndef USE_LOCKFREE_QUEUE
    // gcc编译器可以直接拷贝类中的数组
    inline const Buffer& operator =(const Buffer& src)
    {
        memmove(this->m_data, src.m_data, src.m_length);
        m_length = src.m_length;
        return src;
    }
#endif // USE_LOCKFREE_QUEUE
    inline const uint8_t* data(){
        return static_cast<uint8_t*>(this->m_data);
    }

    inline size_t& size()
    {
        return m_length;
    }

    /**
     * @brief 同vector的resize
     * 
     * @param size resize大小
     * @param n 填充数值，默认0
     * @return int 
     */
    inline size_t resize(size_t size, int n = 0)
    {
        PORT_ASSERT(size <= MAX_BUFFER_SIZE);
        if(size > m_length) {
            memset(this->m_data + m_length, n, size - m_length);
        }
        m_length = size;
        return m_length;
    }

    inline void clear()
    {
        m_length = 0;
    }

    inline bool empty()
    {
        return m_length == 0;
    }

    inline void push_back(uint8_t c)
    {
        if(m_length + 1 >= MAX_BUFFER_SIZE) throw PORT_EXCEPTION("buffer size out of range");
        m_data[m_length] = c;
        ++m_length;
    }

    /**
     * @brief 类似vector的erase，从buffer中删除index对应的位，由后面的位向前补上
     * 
     * @param index 要删除的位
     * @return int -1 删除的是末尾; 原index，但现在对应的是下一个数据
     */
    inline size_t erase(size_t index)
    {
        PORT_ASSERT(index < MAX_BUFFER_SIZE && index >= 0);
        PORT_ASSERT(index < m_length);
        memmove(m_data + index, m_data + index + 1, m_length - index - 1);
        --m_length;
        return index == m_length ? -1 : index;
    }

    std::string toString() 
    {
        std::stringstream ss;
        ss << "{";
        for(auto i = 0; i < m_length; ++i) {
            ss << "0x" << std::hex << (int)m_data[i] << ", ";
        }
        ss << "}";
        return ss.str();
    }

    ~Buffer() = default;

};

#else  // 使用std::vector作为buffer

class Buffer : public std::vector<uint8_t>
{
public:
    Buffer(size_t size = 0) : std::vector<uint8_t>(size)
    {}

    /**
     * @brief 将src拷贝到buffer
     * 
     * @param src 输入
     * @param size 输入大小
     * @return size_t 拷贝字节数
     */
    inline size_t copy(uint8_t* src, size_t size)
    {
        PORT_ASSERT(size > 0);
        this->insert(this->begin(), src, src + size);
        return this->size();
    }

    /**
     * @brief 将buffer拷贝到dst
     * 
     * @param dst 输出
     * @param size 可输出大小，即输出buffer的长度
     * @return size_t 拷贝字节数
     */
    inline size_t copyTo(uint8_t* dst, size_t size)
    {
        PORT_ASSERT(dst != nullptr); 
        size_t bytes_to_copy = std::min(this->size(), size);
        std::copy(this->begin(), this->begin() + bytes_to_copy, dst);
        return bytes_to_copy;
    }

    std::string toString() 
    {
        std::stringstream ss;
        ss << "{";
        for(auto i = 0; i < this->size(); ++i) {
            ss << "0x" << std::hex << (int)this->operator[](i) << ", ";
        }
        ss << "}";
        return ss.str();
    }
};

#endif // USE_STD_VECTOR


/**
 * @brief 类型定义
 * 
 */
// typedef std::vector<uint8_t> Buffer;
typedef std::queue<Buffer> BufferQueue;
typedef struct {Buffer buffer; timeval tv;} BufferWithTime;
typedef std::queue<BufferWithTime> BufferWithTimeQueue;
typedef std::shared_ptr<BufferWithTimeQueue> BufferWithTimeQueuePtr;
typedef struct { Buffer buffer; ID id;} BufferWithID;
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

} // namespace transport

#endif // __BUFFER_HPP__