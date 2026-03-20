 # 启动说明：
 1. 启动雷达驱动：
```
ros2 launch livox_ros_driver2 msg_MID360_launch.py 
```
2. 启动串口
```
ros2 launch myserial myserial.launch.py 
```
3. 启动导航
```
ros2 launch nav_bringup run.launch.py 
```
4. 启动决策
```
ros2 run sentry_decision sentry_decision_node
```