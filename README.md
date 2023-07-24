# Transport
## 类列表
### 基础类
1. BasePackage
   1. 数据结构
      1. BufferWithTime队列
      2. function函数指针，用于向外发包

2. PackageInterFace
   1. 数据结构：无

3. Port
   1. 数据结构
      1. port name 名称
      2. map->(id, base package ptr)收发包映射表
      3. package manager 包管理器
      4. port status 状态描述结构
      5. buffer 收包队列

4. BaseROSInterface
   1. 数据结构：基本ros通信得东西


### 功能类
1. CanPort
   1. 数据结构：继承Port，无额外特殊结构
2. PackageManager
   1. 数据结构
      1. map->(id, base package ptr)收发包映射表
      2. vector->(Port ID table {port name, id list} ) 每个端口名和其负责收发的包
3. PortController
   1. 数据结构
      1. map -> (port name, port ptr) 端口名和对应实例化都指针
      2. map -> (port name, port status) 端口名和对应状态指针
      3. map -> (port name, int) 端口名和其所在的组
4. PortGroup
   1. 数据结构
      1. map-> (port name, port ptr) 端口名和对应的实例化指针