#ifndef __FAKE_PORT__
#define __FAKE_PORT__

#include <iostream>
#include <queue>
#include <WMJProtocol.h>
#include <linux/can/raw.h>

#include "TimeTest.hpp"

#ifdef USE_FAKE
namespace fake
{

void send_process_func(Buffer* buffer, int id);
void recv_process_func(Buffer* buffer, int id);

std::queue<BufferWithID> q;
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
    BufferWithID buffer = q.front();
    if(q.size() > 10) q.pop();
    m.unlock();
    recv_process_func(&buffer.first, buffer.second);
    frame->can_id = buffer.second;
    for(size_t i = 0; i < buffer.first.size(); ++i)
    {
        frame->data[i] = buffer.first[i];
    }
    frame->len = (int)buffer.first.size();
    return CAN_MTU;
}

ssize_t send(int __fd, const void *__buf, size_t __n, int __flags)
{
    (void)__fd;
    (void)__n;
    (void)__flags;
    if(__buf == NULL) return -1;
    canfd_frame* frame = (canfd_frame*)__buf;
    BufferWithID buffer;
    buffer.second = frame->can_id;
    for(int i = 0; i < frame->len; ++i)
    {
        buffer.first.push_back(frame->data[i]);
    }
    ////////
    m.lock();
    q.push(buffer);
    m.unlock();
    send_process_func(&buffer.first, buffer.second);
    return CAN_MTU;
}

void send_process_func(Buffer* buffer, int id) {
    TimeTest t1;
    TimeTest t2;
    switch (id)
    {
    case 0x345:
        std::cout << "t1 " << t1.tv.tv_sec << " " << t1.tv.tv_usec << std::endl;
        std::cout << "t2 " << t2.tv.tv_sec << " " << t2.tv.tv_usec << std::endl;
        t2 << *buffer;
        std::cout << "t2 " << t2.tv.tv_sec << " " << t2.tv.tv_usec << std::endl;
        std::cout << "发包时间：" << t1.getTimeByMicroSec() - t2.getTimeByMicroSec() << "ms" << std::endl;
        break;
    
    default:
        break;
    }
}

void recv_process_func(Buffer* buffer, int id) {
    TimeTest t1;
    switch (id)
    {
    case 0x345:
        *buffer << t1;
        // std::cout << "recv func " << t1.toString() << std::endl;
        break;
    
    default:
        break;
    }
}

}
#endif // USE_FAKE
#endif // __FAKE_PORT__