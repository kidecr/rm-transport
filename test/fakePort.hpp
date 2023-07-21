#ifndef __FAKE_PORT__
#define __FAKE_PORT__

#include <iostream>
#include <queue>
#include <WMJProtocol.h>
#include <linux/can/raw.h>

#ifdef USE_FAKE
namespace fake
{

std::queue<BufferWithID> q;

ssize_t recv(int __fd, void *__buf, size_t __n, int __flags)
{
    (void)__fd;
    (void)__n;
    (void)__flags;
    if(__buf == NULL) return -1;
    if(q.size() <= 0)   // 无包
        return -1;
    canfd_frame* frame = (canfd_frame*)__buf;
    BufferWithID buffer = q.front();
    if(q.size() > 10) q.pop();
    frame->can_id = buffer.second;
    for(int i = 0; i < buffer.first.size(); ++i)
    {
        frame->data[i] = buffer.first[i];
    }
    frame->len = 8;
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
    for(int i = 0; i < 8; ++i)
    {
        buffer.first.push_back(frame->data[i]);
    }
    q.push(buffer);
    return CAN_MTU;
}

}
#endif // USE_FAKE
#endif // __FAKE_PORT__