# log系统使用方法

## log系统简介

该log系统所有代码均在logger.hpp里，主要工作为为ROS自带log、spdlog、glog做了统一封装，提供了统一接口

## log编译

1. 依赖安装：
   1. 当使用ROS自带log作为底层时，无需依赖
   2. 当使用glog作为底层时，使用`apt install libgoogle-glog-dev`安装即可
   3. 当使用spdlog时，推荐使用**编译安装**，使用apt安装或直接headonly安装会有fmt依赖问题，而且导致之后编译很慢。仓库地址https://github.com/gabime/spdlog
2. 编译选项：cmake参数里有`USE_ROS_LOG` `USE_SPD_LOG`，用于设置使用什么log，默认使用glog（ps：glog相比spdlog慢很多，ros的log没法在代码里指定log存储位置，推荐spdlog）

## log用法

1. log系统调用顺序为

   1. 在整个程序开始时，必须先执行`LOGINIT`
      1. 对于使用ROS自带LOG的情况，推荐传入node指针
      2. 对于其他情况，可以不填参数，或者也可以指定名称和存储路径
   2. 随后，即可使用`LOGDEBUG，LOGINFO，LOGWARN，LOGERROR，LOGFATAL`输出log
   3. 参数的格式和printf格式完全，暂时不支持cout的格式或fmt格式

2. log系统移植

   1. 直接拷贝logger.hpp到目标项目位置
   2. 在cmake里find_package所使用的log库，并在链接时添加对应log库即可

3. 示例：

   ```
   LOGINIT(node); // ROS
   LOGINIT();	// 其他
   
   LOGDEBUG("[Debug Print]: buffer id 0x%x : %s", (int)id, buffer.toString().c_str());
   LOGINFO("port %s : write thread exit", port_name.c_str());
   LOGWARN("create port %s failed!", port_name.c_str());
   LOGERROR("Port Manager cannot open config file %s", config_path.c_str());
   // 基本就以上这样
   ```

   