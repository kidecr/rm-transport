#include <Utility.hpp>
#include <iostream>

int main()
{
    timeval t1, t2, t3, t4;
    gettimeofday(&t1, nullptr);
    usleep(1.1e6);
    gettimeofday(&t2, nullptr);
    // gettimeofday(&t3, nullptr);
    t3 = t1 - t2;
    t4 = t2 - t1;
    std::cout << "t1: " << t1 << std::endl;
    std::cout << "t2: " << t2 << std::endl;
    std::cout << "t3: " << t3 << std::endl;
    std::cout << "t4: " << t4 << std::endl;
}