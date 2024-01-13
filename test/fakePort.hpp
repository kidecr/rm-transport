#ifndef __FAKE_PORT__
#define __FAKE_PORT__

#include <iostream>
#include <queue>
#include <linux/can/raw.h>

#include "utils/Utility.hpp"
#include "pkg/TimeTest.hpp"

#ifdef __USE_FAKE__
namespace fake
{

void send_process_func(transport::Buffer* buffer, int id);
void recv_process_func(transport::Buffer* buffer, int id);

std::queue<transport::BufferWithID> q;
std::mutex m;

ssize_t recv(int __fd, void *__buf, size_t __n, int __flags)
{
    (void)__fd;
    (void)__n;
    (void)__flags;
    if(__buf == NULL) return -1;
    if(q.size() <= 0)   // 无包
        return -1;
    canfd_frame* frame = (canfd_frame*)__buf;
    m.lock();
    transport::BufferWithID buffer = q.front();
    while(q.size() > 1) q.pop();
    // q.pop();
    m.unlock();
    recv_process_func(&buffer.buffer, buffer.id);
    frame->can_id = buffer.id;
    for(size_t i = 0; i < buffer.buffer.size(); ++i)
    {
        frame->data[i] = buffer.buffer[i];
    }
    frame->len = (int)buffer.buffer.size();
    return CAN_MTU;
}

ssize_t send(int __fd, const void *__buf, size_t __n, int __flags)
{
    (void)__fd;
    (void)__n;
    (void)__flags;
    if(__buf == NULL) return -1;
    canfd_frame* frame = (canfd_frame*)__buf;
    transport::BufferWithID buffer;
    buffer.id = frame->can_id;
    for(int i = 0; i < frame->len; ++i)
    {
        buffer.buffer.push_back(frame->data[i]);
    }
    ////////
    m.lock();
    q.push(buffer);
    m.unlock();
    send_process_func(&buffer.buffer, buffer.id);
    return CAN_MTU;
}

void send_process_func(transport::Buffer* buffer, int id) {
    transport::TimeTest t1;
    transport::TimeTest t2;
    switch (id)
    {
    case 0x345:
        t2 << *buffer;

        std::cout << "发包时间：" << t1.getTimeByMicroSec() - t2.getTimeByMicroSec() << "ms" << std::endl;
        break;
    // case 0x312:
    //     std::cout << "send functon get 0x312\n";
    default:
        break;
    }
}

void recv_process_func(transport::Buffer* buffer, int id) {
    transport::TimeTest t1;
    transport::TimeTest t2;
    switch (id)
    {
    case 0x345:
        t2 << *buffer;
        t1.index = t2.index;
        *buffer << t1;
        // std::cout << "recv func " << t1.toString() << std::endl;
        break;
    // case 0x312:
    //     std::cout << "recv function get 0x312\n";
    default:
        break;
    }
}

}
#endif // __USE_FAKE__
#endif // __FAKE_PORT__