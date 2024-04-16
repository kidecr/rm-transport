# Package相关使用方法

## PackageManager使用方法
### 设计思路

PackageManager意在提供一个统一的接口，使使用者使用时无需关心收发包涉及哪个Port，只根据ID进行收发。

### 核心接口

1. 发包接口

   ```c++
   template <typename T, IDType T2>
   void send(T2 package_id, T &package)
   ```

   用于发包，两个参数分别为包ID和要发的包，以下为使用样例：

   ```c++
   GimbalPackage gimbal_package;
   /*
   * ... 对gimbal_package的赋值等操作
   */
   m_package_manager->send(GIMBAL, gimbal_package);
   ```

2. 收包接口1

   ```
   template <typename T, IDType T2>
   T recv(T2 package_id)
   ```

   根据给定包ID和包类型，收对应包，并根据给定包类型解码再返回。

   当Port端未收到新包时，会返回最近一次收到的包；当当前时刻与收到上一个包时间间隔超过一秒时，会报warning提示。
   当从未收到该ID对应的包时，会返回给定包类型的默认构造，并输出warning，提示从未收过该包。
   当未在config文件里注册该ID时，会返回给定包类型的默认构造，并输出error，提示未注册该包。以下为使用方法：

   ```c++
   GimbalPackage gimbal_package;
   gimbal_package = m_package_manager->recv<GimbalPackage>(GIMBAL);
   /*
    * ... 对gimbal_package的其他操作
    */
   ```

3. 收包接口2

   ```c++
   template <typename T, IDType T2>
   std::pair<T, timeval> recv(T2 package_id, use_timestamp_t)
   ```

   根据给定包ID和包类型，收对应包，并根据给定包类型解码再返回，同时返回该包接受时刻的时间戳。

   当Port端未收到新包时，会返回最近一次收到的包；当当前时刻与收到上一个包时间间隔超过一秒时，会报warning提示。
   当从未收到该ID对应的包时，会返回给定包类型的默认构造，并输出warning，提示从未收过该包。
   当未在config文件里注册该ID时，会返回给定包类型的默认构造，并输出error，提示未注册该包。以下为使用方法：

   ```c++
   GimbalPackage gimbal_package;
   // 注意：use_timestamp是一个全局变量，声明在PackageManager.hpp里，用的时候直接用就行，不要自行创建
   gimbal_package = m_package_manager->recv<GimbalPackage>(GIMBAL, use_timestamp);
   /*
    * ... 对gimbal_package的其他操作
    */
   ```

### 创建方法

创建时需要将config文件路径作为参数传入构造函数。

```c++
auto packageManager = std::make_shared<PackageManager>(TRANSPORT_CONFIG_FILE_PATH);
```



## 新增pkg方法

### 设计理念

同一类包使用同一个Package类进行管理和实现，对同一类包的定义可以是包结构上相同（data字段结构相同），也可以是逻辑上属于同一类型（如Gimbal）。

每类包对应一个Package类，每个Package类对应一个文件，多个类则创建多个文件，每个文件应包含一个Package类（必须）和一个或多个结构体（可选）。虽然你不这么写我也管不了你。

### 使用方法（对照pkg文件夹里的例子看）

1. 在pkg文件夹里创建新文件

2. 首先应该使用`#define`或`#pragma once`防止重复引用

3. 其次`#include “PackageInterface.hpp”`，其中包含了一个基类和一些工具函数

4. 声明`transport`命名空间

5. 可选：定义一个解析buffer的结构体，命名一般以P开头后跟对应的Package名，写法就照着下面写（格式不一样出问题自己负责）

   ```
   #pragma pack(1)	// 定义对齐方式：按1字节对齐
   struct PGimbal
   {
       uint8_t info : 8;				// 使用':'的语法：设置每个变量的数据位大小
       uint16_t yaw_speed : 12;		// 该结构体的格式应该和你的通信协议的data字段结构一致
       uint16_t pitch_speed : 12;		// 从而实现将buffer按位解析，减轻工作量
       uint16_t yaw_angle : 16;
       uint16_t pitch_angle : 16;
   
       TRANSFORM_FUNC(PGimbal)	// 该宏定义了一系列后续可能需要使用的函数
   };
   #pragma pack() // 结束自定义对齐方式，使用默认对齐（一般是4字节对齐，不确定可以写demo测试一下）
   ```

6. **定义package类**，使用`**Package`命名格式，**使用`public`继承`PackageInterface`类，模板填Package类名**。 **必须要写的**：

   1. **默认构造函数**，根据需要定义拷贝构造函数和=重载。
   2. **encode和decode函数**，发包可以不重载decode，收包可以不重载encode，PackageInterface有默认的。
      1. 参数，encode和decode都接受两个参数，其中buffer就是通信协议中的data字段，另一个参数类型为本身类类型，函数要求显示标记override。
      2. `PackageInterface.hpp`里有一些用于数据变换的函数，可以减轻一定工作量，具体有什么自己去看。
      3. 如果包结构比较复杂，可以定义步骤3中的结构体，通过`target << buffer`将buffer转为结构体，通过`buffer << target`将结构体转成buffer。否则也可以像传统那样做位运算。
   3. toString：用于将Package转成字符串进行输出，注意字符串中非必要尽量不要有换行。这个在log和debug中用，推荐重载，不写也没事，无非是你到时候找问题费点劲罢了。

   **其他部分自由发挥**，可以定义很多其他函数，达到你想实现的功能

   `ps: IENUM和UENUM可以视为常量，不会占用内存空间，有作用域，咋用可以参考代码`
