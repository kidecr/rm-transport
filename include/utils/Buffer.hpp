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

// 设置Buffer最大大小
#ifndef MAX_BUFFER_SIZE
#define MAX_BUFFER_SIZE 128
#endif // MAX_BUFFER_SIZE

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
        PORT_ASSERT(size <= length);
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

    /**
     * @brief 类似vector的erase，从buffer中删除index对应的位，由后面的位向前补上
     * 
     * @param index 要删除的位
     * @return int -1 删除的是末尾; 原index，但现在对应的是下一个数据
     */
    inline int erase(int index)
    {
        PORT_ASSERT(index < MAX_BUFFER_SIZE && index >= 0);
        PORT_ASSERT(index < length);
        memmove(data + index, data + index + 1, length - index - 1);
        --length;
        return index == length ? -1 : index;
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