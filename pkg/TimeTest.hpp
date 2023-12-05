#ifndef __TIME_TEST__
#define __TIME_TEST__

#include "PackageInterface.hpp"

namespace transport{

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

    void decode(TimeTest &tt, Buffer &buffer) override
    {
        if(buffer.size() < 16) {
            LOGWARN("TimeTest recv buffer size less than 16");
            return;
        }
        
        Timeval t;
        t << buffer;
        tt.tv.tv_sec = t.sec;
        tt.tv.tv_usec = t.usec;
        tt.index = t.index;
        return;
    }

    void encode(TimeTest &tt, Buffer &buffer) override
    {
        buffer.resize(sizeof(Timeval), 0);
        Timeval t;
        t.sec = tt.tv.tv_sec;
        t.usec = tt.tv.tv_usec;
        t.index = tt.index;
        buffer << t;
        return;
    }

    std::string toString() override
    {
        std::stringstream sstream;
        sstream << __CLASS__ << std::endl;
        double time = tv.tv_sec * 1e3 + tv.tv_usec * 1e-3;
        sstream << TO_STR(time) << "us" << std::endl;
        return sstream.str();
    }

    double getTimeByMicroSec()
    {
        return tv.tv_sec * 1e3 + tv.tv_usec * 1e-3;
    }
};

} // namespace transport

#endif // __TIME_TEST__