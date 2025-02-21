#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <random> // 引入随机数生成器
#include <atomic>
#define __NOT_USE_LOG__
#include "port/SerialPort.hpp"

constexpr size_t TEST_BUFFER_SIZE = 1024;
constexpr size_t TEST_MAX_RESERVE = 512;

using namespace transport;

class SerialBufferTest : public ::testing::Test {
protected:
    SerialBuffer<TEST_BUFFER_SIZE, TEST_MAX_RESERVE> buffer;
};

// 基本功能测试
TEST_F(SerialBufferTest, InitialState) {
    EXPECT_EQ(buffer.headIndex(), 0);
    EXPECT_EQ(buffer.tailIndex(), 0);
    EXPECT_TRUE(buffer.empty());
}

TEST_F(SerialBufferTest, AppendDataWithinCapacity) {
    uint8_t data[100] = {0};
    buffer.append(data, 100);
    EXPECT_EQ(buffer.tailIndex(), 100);
    EXPECT_EQ(buffer.length(), 100);
}

TEST_F(SerialBufferTest, AppendExactBufferSize) {
    buffer.flush();
    uint8_t data[TEST_BUFFER_SIZE] = {0};
    buffer.append(data, TEST_BUFFER_SIZE - TEST_MAX_RESERVE);
    EXPECT_EQ(buffer.tailIndex(), TEST_BUFFER_SIZE - TEST_MAX_RESERVE);
    EXPECT_EQ(buffer.length(), TEST_BUFFER_SIZE - TEST_MAX_RESERVE);
    buffer.append(data, TEST_MAX_RESERVE);
    EXPECT_EQ(buffer.tailIndex(), TEST_BUFFER_SIZE);
    EXPECT_EQ(buffer.length(), TEST_BUFFER_SIZE);
}

TEST_F(SerialBufferTest, AppendExceedingBufferSize) {
    // 这里会触发断言，测试需在非调试构建下或处理异常
    buffer.flush();
    buffer.append(TEST_BUFFER_SIZE - 1);
    // 验证缓冲区状态是否被正确重置
    EXPECT_EQ(buffer.headIndex(), 0);
    EXPECT_EQ(buffer.tailIndex(), 1023);
    buffer.append(1);
    EXPECT_EQ(buffer.tailIndex(), 1024);
    buffer.setHead(100);
    EXPECT_EQ(buffer.headIndex(), 100);
    EXPECT_EQ(buffer.tailIndex(), 1024);
    buffer.append(100);
    EXPECT_EQ(buffer.headIndex(), 0);
    EXPECT_EQ(buffer.tailIndex(), 100);
    buffer.setHead(100);
    EXPECT_EQ(buffer.headIndex(), 100);
    buffer.append(101);
    EXPECT_EQ(buffer.headIndex(), 100);
    EXPECT_EQ(buffer.tailIndex(), 201);
}

// 测试用例
TEST_F(SerialBufferTest, AppendStressTest) {
    constexpr int NUM_THREADS = 16;
    constexpr int ITERATIONS = 1000;
    std::random_device rd; // 随机数种子
    std::mt19937 gen(rd()); // 使用Mersenne Twister算法的随机数引擎
    std::uniform_int_distribution<> dis_length(1, 100); // 随机数范围 [1, 100]

    // 定义生产者函数
    auto producer = [&]() {
        for (size_t i = 0; i < ITIMER_PROF; ++i) {
            size_t random_length = dis_length(gen); // 随机生成数据长度
            uint8_t random_data[random_length];
            std::generate(random_data, random_data + random_length, [&]() { return static_cast<uint8_t>(gen() % 256); });

            buffer.append(random_data, random_length); // 追加数据
        }
    };

    // 启动多个线程
    std::vector<std::thread> threads;
    for (size_t i = 0; i < NUM_THREADS; ++i) {
        threads.emplace_back(producer);
    }

    // 等待所有线程完成
    for (auto& t : threads) {
        t.join();
    }

    // 验证最终状态
    EXPECT_GE(buffer.tailIndex(), buffer.headIndex()); // 尾部索引应大于等于头部索引
    EXPECT_LE(buffer.length(), 1024); // 缓冲区长度不应超过容量
}

// 边界条件测试
TEST_F(SerialBufferTest, AppendBoundaryTest) {
    constexpr int NUM_THREADS = 16;
    constexpr int ITERATIONS = 1000;
    std::random_device rd; // 随机数种子
    std::mt19937 gen(rd()); // 使用Mersenne Twister算法的随机数引擎
    std::uniform_int_distribution<> dis_length(1, 100); // 随机数范围 [1, 100]


    // 测试刚好填满缓冲区
    size_t total_length = 0;
    while (total_length < 1024) {
        size_t random_length = dis_length(gen);
        if (total_length + random_length > 1024) {
            random_length = 1024 - total_length;
        }
        uint8_t random_data[random_length];
        std::generate(random_data, random_data + random_length, [&]() { return static_cast<uint8_t>(gen() % 256); });

        buffer.append(random_data, random_length);
        total_length += random_length;
    }

    // 验证缓冲区状态
    EXPECT_EQ(buffer.length(), 1024); // 缓冲区应刚好填满

    // 测试超出缓冲区容量
    size_t overflow_length = dis_length(gen);
    uint8_t overflow_data[overflow_length];
    std::generate(overflow_data, overflow_data + overflow_length, [&]() { return static_cast<uint8_t>(gen() % 256); });

    buffer.append(overflow_data, overflow_length);

    // 验证缓冲区重置
    EXPECT_EQ(buffer.headIndex(), 0); // 头部索引应被重置为0
    EXPECT_EQ(buffer.tailIndex(), overflow_length); // 尾部索引应从头开始计算
}

// 性能测试
TEST_F(SerialBufferTest, AppendPerformanceTest) {
    constexpr int NUM_THREADS = 16;
    constexpr int ITERATIONS = 1000;
    std::random_device rd; // 随机数种子
    std::mt19937 gen(rd()); // 使用Mersenne Twister算法的随机数引擎
    std::uniform_int_distribution<> dis_length(1, 100); // 随机数范围 [1, 100]


    size_t total_operations = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < 100000; ++i) {
        size_t random_length = dis_length(gen);
        uint8_t random_data[random_length];
        std::generate(random_data, random_data + random_length, [&]() { return static_cast<uint8_t>(gen() % 256); });

        buffer.append(random_data, random_length);
        total_operations += random_length;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;

    // 输出性能指标
    std::cout << "Total operations: " << total_operations << "\n";
    std::cout << "Elapsed time: " << elapsed_time.count() << " seconds\n";
    std::cout << "Throughput: " << total_operations / elapsed_time.count() / 1024 << " kb/second\n";

    // 验证缓冲区状态
    EXPECT_GE(buffer.tailIndex(), buffer.headIndex());
    EXPECT_LE(buffer.length(), 1024);
}

TEST_F(SerialBufferTest, MemmoveTriggerTest) {
    // 填充数据直到触发memmove
    size_t initialData = TEST_BUFFER_SIZE - TEST_MAX_RESERVE;
    buffer.append(initialData);
    buffer.setHead(initialData - 50);  // 保留50字节在头部
    buffer.append(100);  // 触发memmove
    EXPECT_EQ(buffer.headIndex(), 0);
    EXPECT_EQ(buffer.tailIndex(), 50 + 100);
}

// 边界值测试
TEST_F(SerialBufferTest, HeadTailBoundary) {
    buffer.append(TEST_BUFFER_SIZE - 1);
    buffer.setHead(TEST_BUFFER_SIZE - 1);
    buffer.append(1);  // 应触发重置
    EXPECT_EQ(buffer.headIndex(), 0);
    EXPECT_EQ(buffer.tailIndex(), 1);
}

TEST_F(SerialBufferTest, SetHeadBeyondTail) {
    buffer.append(100);
    buffer.setHead(150);  // 超过当前tail(100)
    EXPECT_EQ(buffer.headIndex(), 100);
}

// 多线程压力测试
TEST_F(SerialBufferTest, MultiThreadAppendConsume) {
    constexpr int NUM_THREADS = 4;
    constexpr int ITERATIONS = 1000;
    std::vector<std::thread> threads;
    std::atomic<bool> start(false);
    // 随机数生成器
    std::random_device rd; // 随机数种子
    std::mt19937 gen(rd()); // 使用Mersenne Twister算法的随机数引擎
    std::uniform_int_distribution<> dis(1, 100); // 随机数范围 [1, 100]

    auto producer = [&]() {
        while (!start.load()) { std::this_thread::yield(); }
        for (int i = 0; i < ITERATIONS; ++i) {
            int randomValue = dis(gen); // 生成随机数
            if (randomValue % 2) {
                uint8_t data[randomValue];
                for (auto i = 0; i < randomValue; ++i){
                    data[i] = dis(gen);
                }
                buffer.append(data, randomValue);
            }
            else{
                buffer.append(randomValue);
            }
        }
    };

    auto consumer = [&]() {
        while (!start.load()) { std::this_thread::yield(); }
        for (int i = 0; i < ITERATIONS; ++i) {
            size_t len = buffer.length();
            if (len > 0) {
                auto randomValue = dis(gen);
                buffer.setHead(randomValue);
            }
        }
    };

    for (int i = 0; i < NUM_THREADS; ++i) {
        threads.emplace_back(producer);
        threads.emplace_back(consumer);
    }

    start.store(true);
    for (auto& t : threads) {
        t.join();
    }

    // 验证最终状态是否合理
    EXPECT_GE(buffer.tailIndex(), buffer.headIndex());
    EXPECT_LE(buffer.length(), TEST_BUFFER_SIZE);
}

// 异常安全测试（假设PORT_ASSERT抛出异常）
TEST_F(SerialBufferTest, InvalidAppend) {
    EXPECT_THROW(buffer.append(nullptr, TEST_BUFFER_SIZE + 1), PortException);
}

// 组合API测试
TEST_F(SerialBufferTest, CombinedAPIOperations) {
    buffer.append(500);
    buffer.setHead(300);
    EXPECT_EQ(buffer.length(), 200);
    buffer.append(300);
    EXPECT_LE(buffer.tailIndex(), TEST_BUFFER_SIZE);
    buffer.flush();
    EXPECT_TRUE(buffer.empty());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    LOGINIT()
    return RUN_ALL_TESTS();
}