# 项目移动至 [Gitlab](http://git.bwbot.org/publish/GalileoSDK)


# GalileoSDK [![Build Status](https://travis-ci.org/BluewhaleRobot/GalileoSDK.svg)](https://travis-ci.org/BluewhaleRobot/GalileoSDK)

Galileo C++ SDK

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
# 安装 cmake,cmake 版本需要3.14以上，如果版本不够需要更新
wget -q https://github.com/Kitware/CMake/releases/download/v3.14.0-rc4/cmake-3.14.0-rc4.tar.gz
tar -xzf cmake-3.14.0-rc4.tar.gz > /dev/null
cd cmake-3.14.0-rc4/
env CC=$(which clang) CXX=$(which clang++) ./bootstrap --prefix=/usr --parallel=4
make
sudo make install
which cmake
cmake --version
cd ..
# 安装ROS依赖包
mkdir -p amd64/src
cd amd64/src
git clone https://github.com/ros/catkin --depth=1
git clone https://github.com/ros/common_msgs --depth=1
git clone https://github.com/ros/gencpp --depth=1
git clone https://github.com/jsk-ros-pkg/geneus --depth=1
git clone https://github.com/ros/genlisp --depth=1
git clone https://github.com/ros/genmsg --depth=1
git clone https://github.com/RethinkRobotics-opensource/gennodejs --depth=1
git clone https://github.com/ros/genpy --depth=1
git clone https://github.com/ros/message_generation --depth=1
git clone https://github.com/ros/message_runtime --depth=1
git clone https://github.com/BluewhaleRobot/ros_comm --depth=1
git clone https://github.com/ros/ros_comm_msgs --depth=1
git clone https://github.com/BluewhaleRobot/rosconsole --depth=1
git clone https://github.com/BluewhaleRobot/roscpp_core --depth=1
git clone https://github.com/ros/std_msgs --depth=1
sudo apt-get install -yqq --allow-unauthenticated python-catkin-pkg python-catkin-tools > /dev/null
cd ..
./src/catkin/bin/catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DBUILD_SHARED_LIBS=OFF -DBoost_USE_STATIC_LIBS=ON -DBoost_LIB_PREFIX=lib -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash
cd ../GalileoSDK
# 安装SDK
cd IotWrapper
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ../../iot
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ../../GalileoSDK
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
sudo make install
cd ../../GalileoSDKTest
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
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
