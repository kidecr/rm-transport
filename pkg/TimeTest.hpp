#ifndef __TIME_TEST__
#define __TIME_TEST__

#include <Package.hpp>

#pragma pack(1)
struct Timeval
{
    long sec;
    long usec;

    TRANSFORM_FUNC(Timeval)
};
#pragma pack()

class TimeTest : public PackageInterFace<TimeTest>
{
public:
    timeval tv;

public:
    TimeTest()
    {
        gettimeofday(&tv, NULL);
    }

    TimeTest(const TimeTest &timetest)
    {
        tv = timetest.tv;
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
        return tt;
    }

    Buffer encode(TimeTest tt) override
    {
        Buffer buffer;
        buffer.resize(16, 0);
        Timeval t;
        t.sec = tt.tv.tv_sec;
        t.usec = tt.tv.tv_usec;
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