# ROS2配置流程  
*这里记录了首次在NUC上配置ROS2，运行哨兵导航的流程*
*在不同ros2版本下安装包时，自行修改成对应版本名*
## 安装ROS2
打开命令行，使用鱼香ROS一键安装ROS2  
>wget http://fishros.com/install -O fishros && . fishros  

*若安装了多个ROS版本，可以修改~/.bashrc进行选择*  

赞美ROS2的新特性，多机通讯不需要设置主从机了，连接在同一局域网即可  

## 安装时间同步工具  
鉴于25赛季哨兵将使用双激光雷达，为了对两台激光雷达进行时间同步，我们需要一个小工具。  
赞美NUC，其自带PTP硬同步，可以简单实现极高精度的时间同步。  
>sudo apt-get install ethtool  
sudo ethtool -T enp114s0  
*//注：enp114s0为网口名*  

软件时间戳需要包括参数  
SOF_TIMESTAMPING_SOFTWARE
SOF_TIMESTAMPING_TX_SOFTWARE
SOF_TIMESTAMPING_RX_SOFTWARE

硬件时间戳需要包括参数  
SOF_TIMESTAMPING_RAW_HARDWARE
SOF_TIMESTAMPING_TX_HARDWARE
SOF_TIMESTAMPING_RX_HARDWARE  

>sudo apt install ptpd  

PTP, 启动！  
>sudo ptpd -M -i enp114s0 -C
## 安装LIVOX_ROS_DRIVER2  
>https://github.com/Livox-SDK/livox_ros_driver2  

由于驱动在安装后基本就不用修改了, 所以可以将其独立出来，放在另一个工作空间下，并设置好环境变量  
编译完成后修改~/.bashrc即可，例如添加这一行  
> source /home/nuc/ws_livox/install/setup.bash

## 其它库的安装  
这里可以参考原来的SentryNX中的文档所写的内容，迁移到ROS2后代码依然依赖这些库  

*TODO： 将sentry_nx.md的内容复制过来*  

## 安装各种ROS中的依赖  

Pointlio依赖：  
>sudo apt-get install ros-humble-pcl-conversions
sudo apt-get install libeigen3-dev
sudo apt-get install ros-foxy-pcl-ros  

>sudo apt-get install ros-foxy-nav2-*  
sudo apt-get install ros-foxy-libg2o  

其他的跟着编译时的报错自行寻找相关包并安装就可以了，不再赘述
此外，在我自己的电脑上编译teb规划器的依赖costmap_converter包时没有发生报错，但迁移到NUC上之后编译提示有两个虚函数没有进行实现，如果这个包不能过编的话，记得这里这个小改动。





