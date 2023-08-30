#ifndef __TIME_TEST__
#define __TIME_TEST__

#include "PackageInterface.hpp"

#pragma pack(1)
struct Timeval
{
    long sec;
    long usec;
    int index;

    TRANSFORM_FUNC(Timeval)
};
#pragma pack()

class TimeTest : public PackageInterFace<TimeTest>
{
public:
    timeval tv;
    int index;
public:
    TimeTest()
    {
        gettimeofday(&tv, NULL);
        index = 0;
    }

    TimeTest(const TimeTest &timetest)
    {
        tv = timetest.tv;
        index = timetest.index;
    }

    constexpr TimeTest &operator=(const TimeTest &timetest) = default;

    TimeTest decode(Buffer buffer) override
    {
        TimeTest tt;
        if(buffer.size() < 16) {
            std::cout << "timetest大小不对 " << buffer.size() << std::endl;
            return tt;
        }
        
        Timeval t;
        t << buffer;
        tt.tv.tv_sec = t.sec;
        tt.tv.tv_usec = t.usec;
        tt.index = t.index;
        return tt;
    }

    Buffer encode(TimeTest tt) override
    {
        Buffer buffer;
        buffer.resize(sizeof(Timeval), 0);
        Timeval t;
        t.sec = tt.tv.tv_sec;
        t.usec = tt.tv.tv_usec;
        t.index = tt.index;
        buffer << t;
        return buffer;
    }

    std::string toString() override
    {
        std::stringstream sstream;
        sstream << typeid(*this).name() << std::endl;
        double time = tv.tv_sec * 1e3 + tv.tv_usec * 1e-3;
        sstream << TO_STR(time) << "us" << std::endl;
        return sstream.str();
    }

    double getTimeByMicroSec()
    {
        return tv.tv_sec * 1e3 + tv.tv_usec * 1e-3;
    }
};

#endif // __TIME_TEST__