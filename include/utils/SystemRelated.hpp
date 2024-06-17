#ifndef __SYSTEM_RELATED_HPP__
#define __SYSTEM_RELATED_HPP__

#include <cstring>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sched.h>
#include <filesystem>

/**
 * @brief 设置线程内核绑定(亲和度)
 * 
 * @param cpu_id 目标内核
 */
inline void set_cpu_affinity(int cpu_id) {
    cpu_set_t set;
    CPU_ZERO(&set);
    // 设置绑定的核
    CPU_SET(cpu_id, &set);
    // 设置绑定当前进程
    sched_setaffinity(0, sizeof(cpu_set_t), &set);
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
}
#endif // __SYSTEM_RELATED_HPP__