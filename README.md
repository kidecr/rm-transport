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

## 改进

1. PortGroup 整合到PortController里，统一叫PortManager，给PackageManager提供绑定回调函数绑定服务
2. PortManager记录是否开启Controller
3. PortManager只负责更新Package的回调函数，PackageManager不负责管理底层哪个Package对应到哪个Port了，这个PortManager管
4. 检测到崩溃，调度方法：依次从损坏的port中获取unorder_map，申请一个新的相同pkg，绑定函数放到PackageManager里，另原来的Port不崩溃。
5. 初始化时，哪个Pacakge对应哪个Port由PortManager读取
6. 首先申请PackageManager，然后申请PortManager，然后portmanager->register(packagemanager),对packagemanager里都包依次迭代，去找对应哪个port，依次注册包，如果没找到默认can0

## 改进

0. Port：增加线程执行一次read和write都接口，线程函数不再暴露在外，usleep改进和
1. CanPort
   1. 数据结构：继承Port，无额外特殊结构，（接口不可用后，改进usleep时间，线程不再崩溃，设置定时检测函数）
2. PackageManager
   1. 数据结构
      1. map->(id, base package ptr)收发包映射表
      2. vector->(Port ID table {port name, id list} ) 每个端口名和其负责收发的包{这个转移到portManager里}
3. PortController
   1. 数据结构
      1. map -> (port name, port ptr) 端口名和对应实例化都指针
      2. map -> (port name, port status) 端口名和对应状态指针（可以不要了）
      3. map -> (port name, int) 端口名和其所在的组 // 这个不要了
      4. 
4. PortGroup
   1. 数据结构
      1. map-> (port name, port ptr) 端口名和对应的实例化指针（不要了）