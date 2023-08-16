#ifndef __PACKAGE_INTERFACE_HPP__
#define __PACKAGE_INTERFACE_HPP__

#include <iostream>
#include <string>

#include "BasePackage.hpp"
#include "Utility.hpp"

template <typename T>
class PackageInterFace //: public BasePackage
{
public:
    using SharedPtr = std::shared_ptr<PackageInterFace<T>>;

public:
    virtual T decode(Buffer buffer)
    {
        (void)buffer;
        T target;
        return target;
    }
    virtual Buffer encode(T target)
    {
        (void)target;
        Buffer buffer;
        return buffer;
    }

    friend void operator>>(T &package, Buffer &buffer)
    {
        buffer = package.encode(package);
    }

    friend void operator>>(Buffer &buffer, T &package)
    {
        package = package.decode(buffer);
    }

    friend void operator<<(T &package, Buffer &buffer)
    {
        package = package.decode(buffer);
    }

    friend void operator<<(Buffer &buffer, T &package)
    {
        buffer = package.encode(package);
    }

    virtual std::string toString()
    {
        std::string str = typeid(*this).name();
        return str;
    }

};

#endif // __PACKAGE_INTERFACE_HPP__