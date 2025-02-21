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

#if !defined(USE_UNQIUE_PTR) && !defined(USE_STD_VECTOR)

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

#elif defined(USE_UNQIUE_PTR)

class Buffer {
private:
    std::unique_ptr<uint8_t[]> m_data; // 使用 unique_ptr 管理动态内存
    size_t m_length;                  // 当前数据长度
    size_t m_capacity;                // 内存容量

public:
    /**
     * @brief 构造函数，初始化缓冲区大小
     * 
     * @param size 初始时buffer大小 
     * @param capacity buffer最大空间
     */
    Buffer(size_t size = 0, size_t capacity = MAX_BUFFER_SIZE) 
    : m_data(std::make_unique<uint8_t[]>(capacity)), m_length(size), m_capacity(capacity) 
    {}

    /**
     * @brief 拷贝构造函数，用于深拷贝
     * 
     * @param buffer 源buffer
     */
    Buffer(const Buffer& buffer) 
    : m_data(std::make_unique<uint8_t[]>(buffer.m_capacity)), m_length(buffer.m_length), m_capacity(buffer.m_capacity) 
    {
        if (buffer.m_data) {
            std::memcpy(m_data.get(), buffer.m_data.get(), buffer.m_length);
        }
    }

    /**
     * @brief 移动构造函数，用于浅拷贝
     * 
     * @param buffer 源buffer，拷贝后源buffer失去可访问内存
     */
    Buffer(Buffer&& buffer) noexcept 
    : m_data(std::move(buffer.m_data)), m_length(buffer.m_length), m_capacity(buffer.m_capacity) 
    {
        buffer.m_length = 0;
        buffer.m_capacity = 0;
    }

    /**
     * @brief 深拷贝方法，拷贝内存
     * 
     * @return Buffer 返回新buffer
     */
    Buffer deepCopy() const {
        Buffer newBuffer(m_length, m_capacity);
        if (m_data) {
            std::memcpy(newBuffer.m_data.get(), m_data.get(), m_length);
        }
        return newBuffer;
    }

    /**
     * @brief 移动赋值运算符，使用移动语义进行浅拷贝
     * 
     * @param other 源buffer
     * @return Buffer& 
     */
    Buffer& operator=(Buffer&& other) noexcept {
        if (this != &other) {
            m_data = std::move(other.m_data);
            m_length = other.m_length;
            m_capacity = other.m_capacity;
            other.m_length = 0;
            other.m_capacity = 0;
        }
        return *this;
    }

    /**
     * @brief 拷贝赋值运算符，使用深拷贝进行赋值
     * 
     * @param other 源buffer
     * @return Buffer& 
     */
    Buffer& operator=(const Buffer& other) {
        if (this != &other) {
            m_data = std::make_unique<uint8_t[]>(other.m_capacity);
            m_length = other.m_length;
            m_capacity = other.m_capacity;
            if (other.m_data) {
                std::memcpy(m_data.get(), other.m_data.get(), other.m_length);
            }
        }
        return *this;
    }
    /**
     * @brief 将 src 数据拷贝到当前缓冲区
     * 
     * @param src 源数据，该函数不判断src的合法性
     * @param size 拷贝长度
     * @return size_t buffer新长度
     */
    inline size_t copy(uint8_t* src, size_t size) {
        PORT_ASSERT(size <= m_capacity && size > 0);
        std::memcpy(m_data.get(), src, size);
        m_length = size;
        return m_length;
    }

    /**
     * @brief 将当前缓冲区的数据拷贝到 dst
     * 
     * @param dst 目标位置，该函数不判断dst的合法性
     * @param size 拷贝长度
     * @return size_t 拷贝长度
     */
    inline size_t copyTo(uint8_t* dst, size_t size) const {
        PORT_ASSERT(size <= m_length);
        std::memcpy(dst, m_data.get(), size);
        return size;
    }

    /**
     * @brief 通过索引访问元素
     * 
     * @param index 索引，要求不越界
     * @return uint8_t& 该位置至的引用
     */
    inline uint8_t& operator[](size_t index) {
        PORT_ASSERT(index < m_length && index >= 0);
        return m_data[index];
    }

    /**
     * @brief 通过索引访问元素, 常量版本
     * 
     * @param index 索引，要求不越界
     * @return const uint8_t& 该位置至的引用
     */
    inline const uint8_t& operator[](size_t index) const {
        return const_cast<const uint8_t&>(const_cast<Buffer*>(this)->operator[](index));
    }

    /**
     * @brief 返回指向底层数据的指针
     * 
     * @return const uint8_t* 指针
     */
    inline const uint8_t* data() const {
        return m_data.get();
    }

    /**
     * @brief 返回当前数据长度
     * 
     * @return size_t 数据长度
     */
    inline size_t size() const {
        return m_length;
    }

    /**
     * @brief 返回当前最大容量
     * 
     * @return size_t 最大容量
     */
    inline size_t capacity() const {
        return m_capacity;
    }

    /**
     * @brief 调整buffre长度并填充默认值
     * 
     * @param size 目标大小
     * @param n 填充默认值
     * @return size_t 最终大小
     */
    inline size_t resize(size_t size, int n = 0) {
        PORT_ASSERT(size <= m_capacity && size >= 0);
        if (size > m_length) {
            std::memset(m_data.get() + m_length, n, size - m_length);
        }
        m_length = size;
        return m_length;
    }

    /**
     * @brief 清空缓冲区
     * 
     */
    inline void clear() {
        m_length = 0;
    }

    /**
     * @brief 判断缓冲区是否为空
     * 
     * @return true 空
     * @return false 存在有效数据
     */
    inline bool empty() const {
        return m_length == 0;
    }

    /**
     * @brief 在缓冲区末尾添加一个字节，注意不可以超过最大缓冲区容量
     * 
     * @param c 待添加字符
     */
    inline void push_back(uint8_t c) {
        if (m_length + 1 > m_capacity) throw PORT_EXCEPTION("buffer size out of range");
        m_data[m_length] = c;
        ++m_length;
    }

    /**
     * @brief 类似vector的erase，从buffer中删除index对应的位，由后面的位向前补上
     * 
     * @param index 要删除的位
     * @return int -1 删除的是末尾; 原index，但现在对应的是下一个数据
     */
    inline size_t erase(size_t index) {
        PORT_ASSERT(index < m_length && index >= 0); // 确保索引在有效范围内
        if (index == m_length - 1) {   // 如果要删除的是最后一个元素
            --m_length;                // 直接减少长度，不需要移动内存
            return -1;                 // 返回 -1 表示删除的是末尾元素
        }
        std::memmove(m_data.get() + index, m_data.get() + index + 1, m_length - index - 1); // 移动后续元素
        --m_length; // 减少长度
        return index; // 返回新的索引位置
    }

    /**
     * @brief 将缓冲区内容转换为字符串表示
     * 
     * @return std::string 
     */
    std::string toString() const {
        std::stringstream ss;
        ss << "{";
        for (auto i = 0; i < m_length; ++i) {
            ss << "0x" << std::hex << static_cast<int>(m_data[i]) << ", ";
        }
        ss << "}";
        return ss.str();
    }

    ~Buffer() = default;
};

#else // 使用std::vector作为buffer

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