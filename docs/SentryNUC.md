


## 4. 导航环境配置
### 4.1. ROS
建议采用鱼香ROS一键安装
```shell
wget http://fishros.com/install -O fishros && . fishros
```

### 4.2. 安装导航相关的包
安装livox_ros_driver2(文件里面已经有了)
**注意ip地址配置：**
要将config/MID360_config里面的`host_net_info`的ip都改为本机的ip,·将`lidar_configs`的ip改为`192.168.1.1xx`,xx的位置请用雷达标签上的后两位来替代。
  安装Livox-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS="-include cstdint -std=c++17"
make -j20
sudo make install
```
*安装acados（从依赖包里下载）*
```
cd acados
git submodule update --recursive --init 下载git下对应的依赖
mkdir build
cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DACADOS_WITH_QPOASES=ON \
    -DACADOS_INSTALL_DIR=/usr/local
make -j20
sudo make install
sudo ldconfig
```

> 备注（重要）：上面是历史安装方式。  
> 当前导航仓导出 `model_ocp/export_ocp.py` 时，推荐使用与 `acados_template` 同源的一致前缀：
> `-DACADOS_INSTALL_DIR=$HOME/dependency/acados`，并在导出前设置
> `ACADOS_SOURCE_DIR=$HOME/dependency/acados`、`LD_LIBRARY_PATH=$HOME/dependency/acados/lib:$LD_LIBRARY_PATH`。  
> 否则容易出现导出阶段符号不匹配（如 `undefined symbol: ocp_nlp_dump_last_qp_to_json`）。
```shell

sudo apt install libsdl-image1.2-dev
sudo apt install libsdl1.2-dev
sudo apt install ros-jazzy-geographic-info ros-jazzy-geographic-msgs
sudo apt install ros-jazzy-serial-driver
sudo apt install ros-jazzy-libg2o
sudo apt install ros-jazzy-gtsam
```

💡 进阶技巧：如何避免“葫芦娃救爷爷”？（强烈推荐）

你现在遇到了缺 pcl_ros 的报错，装完这个之后，再次编译可能又会报错说缺另一个包（比如缺 nav_msgs 或 tf2）。这种一个一个手动装非常痛苦。

ROS 提供了一个一键自动安装所有缺失依赖的神器：rosdep。

建议你在你的工作空间根目录下（也就是有 src 文件夹的那个目录，你敲 colcon build 或 catkin_make 的地方），运行这行命令：
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```  
  修改CMakeLists.txt,删除关于ros1的全部内容（删除if-else,以及if里面的全部内容）
  
  安装fmt

可以使用下面这个编译语句：
```
colcon build --parallel-workers 2 --executor sequential
```
这样更安全，内存不会炸

编译的时候缺的东西再下

### 4.3安装其他需要的软件
- vscode
- 向日葵远程桌面
- chorome
- clash verge等等