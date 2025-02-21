#define USE_UNQIUE_PTR
#include "utils/Buffer.hpp"
#include <gtest/gtest.h>

using namespace transport;

TEST(BufferTest, DefaultConstructor) {
    Buffer buf;
    EXPECT_EQ(buf.size(), 0);
    EXPECT_EQ(buf.capacity(), 128);
}

TEST(BufferTest, ParameterizedConstructor) {
    Buffer buf(5, 10);
    EXPECT_EQ(buf.size(), 5);
    EXPECT_EQ(buf.capacity(), 10);
}

TEST(BufferTest, CopyData) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    Buffer buf(5, 10);
    buf.copy(data, 5);
    EXPECT_EQ(buf.size(), 5);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(buf[i], data[i]);
    }
}

TEST(BufferTest, CopyTo) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    Buffer buf(5, 10);
    buf.copy(data, 5);
    uint8_t dest[5];
    buf.copyTo(dest, 5);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(dest[i], data[i]);
    }
}

TEST(BufferTest, OperatorBracket) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    EXPECT_EQ(buf[0], 0x01);
    EXPECT_EQ(buf[1], 0x02);
    EXPECT_EQ(buf[2], 0x03);
    EXPECT_EQ(buf[3], 0x04);
    EXPECT_EQ(buf[4], 0x05);
}

TEST(BufferTest, ConstOperatorBracket) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    Buffer buf(5, 10);
    buf.copy(data, 5);
    const Buffer& constBuf = buf;
    EXPECT_EQ(constBuf[0], 0x01);
    EXPECT_EQ(constBuf[1], 0x02);
    EXPECT_EQ(constBuf[2], 0x03);
    EXPECT_EQ(constBuf[3], 0x04);
    EXPECT_EQ(constBuf[4], 0x05);
}

TEST(BufferTest, DataPointer) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    Buffer buf(5, 10);
    buf.copy(data, 5);
    const uint8_t* ptr = buf.data();
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(ptr[i], data[i]);
    }
}

TEST(BufferTest, SizeAndCapacity) {
    Buffer buf(5, 10);
    EXPECT_EQ(buf.size(), 5);
    EXPECT_EQ(buf.capacity(), 10);
}

TEST(BufferTest, Resize) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    buf.resize(10, 0xFF);
    EXPECT_EQ(buf.size(), 10);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(buf[i], data[i]);
    }
    for (size_t i = 5; i < 10; ++i) {
        EXPECT_EQ(buf[i], 0xFF);
    }
}

TEST(BufferTest, Clear) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    buf.clear();
    EXPECT_EQ(buf.size(), 0);
}

TEST(BufferTest, Empty) {
    Buffer buf;
    EXPECT_TRUE(buf.empty());
    buf.push_back(0x01);
    EXPECT_FALSE(buf.empty());
}

TEST(BufferTest, PushBack) {
    Buffer buf(0, 10);
    buf.push_back(0x01);
    buf.push_back(0x02);
    buf.push_back(0x03);
    EXPECT_EQ(buf.size(), 3);
    EXPECT_EQ(buf[0], 0x01);
    EXPECT_EQ(buf[1], 0x02);
    EXPECT_EQ(buf[2], 0x03);
}

TEST(BufferTest, EraseMiddleElement) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    size_t newIndex = buf.erase(2);
    EXPECT_EQ(newIndex, 2);
    EXPECT_EQ(buf.size(), 4);
    EXPECT_EQ(buf[0], 0x01);
    EXPECT_EQ(buf[1], 0x02);
    EXPECT_EQ(buf[2], 0x04);
    EXPECT_EQ(buf[3], 0x05);
}

TEST(BufferTest, EraseLastElement) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    size_t newIndex = buf.erase(4);
    EXPECT_EQ(newIndex, -1);
    EXPECT_EQ(buf.size(), 4);
    EXPECT_EQ(buf[0], 0x01);
    EXPECT_EQ(buf[1], 0x02);
    EXPECT_EQ(buf[2], 0x03);
    EXPECT_EQ(buf[3], 0x04);
}

TEST(BufferTest, MoveConstructor) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    Buffer movedBuf(std::move(buf));
    EXPECT_EQ(movedBuf.size(), 5);
    EXPECT_EQ(movedBuf.capacity(), 10);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(movedBuf[i], data[i]);
    }
    EXPECT_EQ(buf.size(), 0);
    EXPECT_EQ(buf.capacity(), 0);
}

TEST(BufferTest, MoveAssignment) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    Buffer movedBuf;
    movedBuf = std::move(buf);
    EXPECT_EQ(movedBuf.size(), 5);
    EXPECT_EQ(movedBuf.capacity(), 10);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(movedBuf[i], data[i]);
    }
    EXPECT_EQ(buf.size(), 0);
    EXPECT_EQ(buf.capacity(), 0);
}

TEST(BufferTest, NoMoveAssignment) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    Buffer movedBuf;
    movedBuf = buf;
    EXPECT_EQ(movedBuf.size(), 5);
    EXPECT_EQ(movedBuf.capacity(), 10);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(movedBuf[i], data[i]);
    }
    EXPECT_EQ(buf.size(), 5);
    EXPECT_EQ(buf.capacity(), 10);
}

TEST(BufferTest, DeepCopy) {
    Buffer buf(5, 10);
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    buf.copy(data, 5);
    Buffer copiedBuf = buf.deepCopy();
    EXPECT_EQ(copiedBuf.size(), 5);
    EXPECT_EQ(copiedBuf.capacity(), 10);
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(copiedBuf[i], data[i]);
    }
    // Modify original buffer to ensure deep copy
    buf[0] = 0xFF;
    EXPECT_EQ(buf[0], 0xFF);
    EXPECT_EQ(copiedBuf[0], 0x01);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}