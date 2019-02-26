# GalileoSDK

Galileo C++ SDK
[![Build Status](https://travis-ci.org/uos/rospy_message_converter.svg)](https://travis-ci.org/BluewhaleRobot/GalileoSDK)

伽利略导航系统C++ SDK，实现了[伽利略导航系统协议](https://doc.bwbot.org/en/books-online/galileo-proto/)

`注意目前只支持x64平台`

## 编译

### 在Windows编译

下载源代码

```ps
git clone https://github.com/bluewhalerobot/GalileoSDK
```

用Visual Studio打开GalileoSDK.sln，编译项目
dll生成在x64文件夹

### 在Linux编译

```bash
git clone https://github.com/bluewhalerobot/GalileoSDK
cd GalileoSDK/GalileoSDK
mkdir build
cd build
cmake ..
make -j
sudo make install
```

## 使用

### 在Windows上使用

可以使用自己从源代码编译的dll也可以使用Release页面域编译的dll。
把include文件夹添加到自己的项目的头文件路径中。把lib文件夹中的.lib文件添加到连接器路径中即可。

### 在Linux上使用

在安装完成后，系统中会添加GalileoSDK。可以在自己的项目中通过CMakeLists.txt文件进行引用。

```cmake
find_package(GalileoSDK)

# 添加链接库路径
target_link_libraries( xxx
    GalileoSDK_LIBRARIES
)

# 添加头文件路径
include_directories( xxx
    GalileoSDK_INCLUDE_DIR
)
```

### 调用例子

下面的例子向机器人发布测试消息。程序运行后可以在机器人上创建`/pub_test`话题

```cpp

#include <GalileoSDK/GalileoSDK.h>
#include <GalileoSDK/galileo_serial_server/GalileoStatus.h>

int main()
{
    GalileoSDK::GalileoSDK sdk;
    while (true)
    {
        auto servers = sdk.GetServersOnline(); // 获取当前在线的机器人
        if(servers.size() == 0){
            std::cout << "No server found" << std::endl;
        }
        for (auto it = servers.begin(); it < servers.end(); it++)
        {
            std::cout << it->getID() << std::endl;
            sdk.Connect("", true, 10000, NULL, NULL); // 连接机器人
        }
        sdk.PublishTest(); // 发布测试消息
        sleep(1);
    }
}

```