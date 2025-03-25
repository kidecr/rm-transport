#ifndef __SYSTEM_RELATED_HPP__
#define __SYSTEM_RELATED_HPP__

#include <cstring>
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <filesystem>

#ifdef _WIN32
// 实现windows下对gettimeofday的替代
#define NOGDI
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <windows.h>
#include <stdint.h>
#include <io.h>

#ifndef F_OK
#define F_OK 0
#endif 
#ifndef R_OK
#define R_OK 2
#endif 
#ifndef W_OK
#define W_OK 4
#endif 
#ifndef X_OK
#define X_OK 6
#endif 


// #ifndef TIMEVAL_STRUCT
// #define TIMEVAL_STRUCT
// struct timeval {
//     long tv_sec;
//     long tv_usec;
// };
// #endif // TIMEVAL_STRUCT

inline int gettimeofday(struct timeval* tv, void* /*tz*/) {
    constexpr uint64_t EPOCH_DIFFERENCE = 116444736000000000ULL; // 1601~1970 的 100ns 间隔数

    FILETIME ft;
    GetSystemTimeAsFileTime(&ft); // 获取当前 UTC 时间

    // 将 FILETIME 转换为 64 位整型
    ULARGE_INTEGER uli;
    uli.LowPart  = ft.dwLowDateTime;
    uli.HighPart = ft.dwHighDateTime;

    // 转换为 Unix 时间戳（1970 起）
    uint64_t timeSince1970 = (uli.QuadPart - EPOCH_DIFFERENCE) / 10; // 转换为微秒

    tv->tv_sec  = static_cast<long>(timeSince1970 / 1000000); // 秒
    tv->tv_usec = static_cast<long>(timeSince1970 % 1000000); // 微秒

    return 0;
}
#else
#include <sys/time.h> 
#include <sys/stat.h>
#include <unistd.h>
#include <sched.h>
#endif


/**
 * @brief 设置线程内核绑定(亲和度)
 * 
 * @param cpu_id 目标内核编号
 */
inline void set_cpu_affinity(int cpu_id) {
#if defined(_WIN32)
    // Windows 使用位掩码设置亲和性
    DWORD_PTR mask = 1ULL << cpu_id;
    HANDLE hThread = GetCurrentThread();
    SetThreadAffinityMask(hThread, mask);
#else
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu_id, &set);
    sched_setaffinity(0, sizeof(cpu_set_t), &set);
#endif
}


/**
 * @brief 重定向标准输入输出流
 * 
 * @param file 目标流
 * @param stream_flag STDIN_FILENO 标准输入 STDOUT_FILENO 标准输出 STDERR_FILENO 标准错误输出
 * @return int 0 成功重定向 -1 打开文件失败 -2 重定向失败
 */
int bind_stdio_to(char* file, int stream_flag)
{
#if defined(_WIN32)
    // 基于windows CRT实现 
    int fd;
    int mode = _O_TEXT; // 文本模式

    // 设置打开模式
    if (stream_flag == 0) { // STDIN
        mode |= _O_RDONLY;
    } else { // STDOUT/STDERR
        mode |= _O_WRONLY | _O_CREAT | _O_TRUNC;
    }

    // 转换路径格式
    fd = _open(file, mode, _S_IREAD | _S_IWRITE);
    if (fd == -1) {
        perror("open file failed: ");
        return -1;
    }

    // 重定向标准流
    if (_dup2(fd, stream_flag) == -1) {
        perror("rebind failed: ");
        _close(fd);
        return -2;
    }

    // 刷新缓冲区
    if (stream_flag != 0) {
        FILE* fp = (stream_flag == 1) ? stdout : stderr;
        freopen(file, (stream_flag == 1) ? "w" : "a", fp);
        setvbuf(fp, NULL, _IONBF, 0); // 禁用缓冲，若存在高频输出，可以将缓冲设为1024或4096对应1K和4KB缓冲
    }

    _close(fd); // 原始描述符不再需要
    return 0;
#else
    int oflag = O_RDONLY;
    if(stream_flag > 0) oflag = O_WRONLY;
    int fd = open(file, oflag);  // 打开目标终端设备，只读模式
    if (fd == -1) {
        // 打开文件失败
        perror("open file failed: ");
        return -1;
    }

    // 将目标终端设备的文件描述符复制给标准输入
    if (dup2(fd, stream_flag) == -1) {
        // 重定向失败
        perror("rebind failed: ");
        close(fd);
        return -2;
    }
    return 0;
#endif
}

/**
 * @brief 清空内存
 * @param t 目标指针
 */
template <typename T>
inline void clear(T *t)
{
    memset(t, 0, sizeof(T));
}

/**
 * @brief 获取当前时间
 * 
 * @return timeval 
 */
inline timeval gettimeval()
{
    timeval tv;
    gettimeofday(&tv, nullptr);
    return tv;
}

/**
 * @brief 判断路径是否是文件夹
 * 
 * @param directory 
 * @return true 
 * @return false 
 */
bool isFolder(const char* directory)
{
    std::filesystem::path path(directory);
    return std::filesystem::is_directory(path);
}

/**
 * @brief 判断路径是否是常规文件
 * 
 * @param directory 
 * @return true 
 * @return false 
 */
bool isFile(const char* directory)
{
    std::filesystem::path path(directory);
    return std::filesystem::is_regular_file(path);
}

/**
 * @brief 判断路径是否存在
 * 
 * @param directory 
 * @return true 
 * @return false 
 */
bool isExist(const char* directory)
{
    std::filesystem::path path(directory);
    return std::filesystem::exists(path);
}

/**
 * @brief 判断路径是否是字符型文件
 * 
 * @param directory 
 * @return true 
 * @return false 
 */
bool isCharacterFile(const char* directory)
{
    std::filesystem::path path(directory);
    return std::filesystem::is_character_file(path);
}

/**
 * @brief 创建多级目录
 * 
 * @param directory 目录
 * @return true 
 * @return false 
 */
bool createDirectory(const char *directory)
{
    std::filesystem::path path(directory);
    if(!std::filesystem::exists(path))
    {
        bool created = std::filesystem::create_directories(path);
        return created;
    }
    return true;
}
/**
 * @brief 执行命令并返回执行结果
 * 
 * @param command 输入命令
 * @return std::vector<std::string> 命令输出
 */
std::vector<std::string> execCommand(const std::string& command) {  
#if defined(_WIN32)
    // 基于windows CRT
    std::vector<std::string> output;
    FILE* pipe = _popen(command.c_str(), "r");
    if (!pipe) {
        perror("_popen() failed");
        return output;
    }

    try {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            output.push_back(buffer);
        }
    } catch (...) {
        _pclose(pipe);
    }

    _pclose(pipe);
    return output;
#else
    std::vector<std::string> output;  
  
    FILE* pipe = popen(command.c_str(), "r");  
    if (!pipe) {   
        perror("popen() failed");
        return output;  
    }  
  
    char buffer[128];  
    while (!feof(pipe)) {  
        if (fgets(buffer, 128, pipe) != nullptr) {  
            output.push_back(buffer);  
        }  
    }  
    pclose(pipe);  
    return output;  
#endif
}
#endif // __SYSTEM_RELATED_HPP__