# Localization Initializer
## 该算法简介
该算法采用了开源项目ndt_omp，来进行初始的位姿匹配，经检验，该算法在开启了多线程和编译优化后，能在5秒左右得到结果，在初始位姿选择大致准确是，此算法能得到较好的效果，输出odom->map的tf
## 使用教程
1. 先编译该文件
2. 打开一个终端，运行point_lio
```
ros2 launch small_point_lio small_point_lio.launch.py
```
3. 打开另一个终端，运行ndt
```
source install/setup.bash
ros2 launch localization_initalizer localization_init.launch.py
```
这个时候会打开rviz,会看到一张先验的地图
使用"2D Pose Estimate"工具，选出机器人在地图里对应的位姿
4. 等待结果
## 参数调整参考
1. 经过使用，发现只开距离过滤的优化，效果是最好的，所以删除了其他的优化只保留了距离筛选。
请注意调整高度，尽量把地板和天花版给过滤了
其他的请参考参数配置文件
## 后续的优化
1. 引入Patchwork++,现在使用仍然有许多bug，后续解决了之后，能很好的过虑掉地板
2. 引入一些动态障碍物过滤算法，过滤掉场地里的人
3. 将转化后的tf接入导航