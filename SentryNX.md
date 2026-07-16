# SentryNX

## 1. 重装系统
 1. 首先，拔掉NX的供电线

 2. 在ubuntu系统打开`SDKManager`软件

 3. 按住NX上的`REC`键，同时插入供电线

 4. 进入STEP1，应当会识别到我们的NX

    ![STEP1](reset_step1.png)

 5. 按照图片设定STEP2，切记只选中Linux
    ![STEP2](reset_step2.png)

 6. 进入STEP3，按照图片STEP3选中选项
    ![STEP3](reset_step3.png)

 7. 安装完毕即可`Finish and Exit`



## 2. 基础环境配置

### 2.1. apt及系统更新

- 进入apt源配置文件

  ```shell
  sudo vi /etc/apt/sources.list
  ```

- 在末尾添加以下内容后保存退出

  ```shell
  deb http://ppa.launchpad.net/ubuntu-toolchain-r/test/ubuntu bionic main 
  deb-src http://ppa.launchpad.net/ubuntu-toolchain-r/test/ubuntu bionic main
  ```

- 添加apt-key(ubuntu-20.04版本,其他版本请自行查找,<https://launchpad.net/~ubuntu-toolchain-r/+archive/ubuntu/test?field.series_filter=trusty,并替换指令中的序列码>)

  ```shell
  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 60C317803A41BA51845E371A1E9377A2BA9EF27F
  ```

- 更新apt

  ```shell
  sudo apt update
  sudo apt upgrade
  ```
  这里经常会出问题，建议跳转到4.1，通过鱼香ROS自动配置源

### 2.2.  g++, gcc安装

  ```shell
  sudo apt install gcc-10 g++-10
  ```

  ```shell
  sudo vi /usr/bin
  ```

  ```shell
  sudo rm g++
  sudo rm gcc
  sudo ln -s g++-10 g++
  sudo ln -s gcc-10 gcc
  ```

- 检查版本

  ```shell
  g++ --version
  ```

### 2.3. cmake安装

  ```shell
  sudo apt-get install libssl-dev
  sudo apt install cmake
  ```

- 检查版本

  ```shell
  cmake --version
  ```

### 2.4. jtop安装，及功率模式设置

```shell
sudo apt install python3-pip
```

```shell
sudo -H pip3 install -U jetson-stats
```

- jtop需要重启生效，为了方便起见，首先在桌面右上角`15W`字样处点击设置功率模式25W，再确认重启。重启后终端输入`jtop`查看各种信息，jtop在下侧导航栏切换信息界面，其中`7 INFO`中包含了我们需要的jetpack、opencv、cuda等版本信息，`ESC`或点击`8 Quit`退出jtop。另外如有需要，jtop中可以控制风扇转速（点击）。

### 2.5. jetpack

- jetpack版本默认选择35.5，安装过程中会提示是否安装，选择`Y`即可

  ```shell
  sudo apt install nvidia-jetpack
  ```

- 安装大小9G左右，请耐心等待。

- 配置cuda环境变量

  ```shell
  sudo vi ~/.bashrc
  ```

- 在末尾添加以下内容后保存退出

  ```shell
  export PATH=/usr/local/cuda/bin:$PATH
  export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
  ```

- 使环境变量生效

  ```shell
  source ~/.bashrc
  ```

- 安装结束后，jtop中`7 INFO`中的jetpack版本应该为35.5，对应各种库已经安装完成。（OPENCV的CUDA加速会是MISSING，无需担心）

### 2.6. Eigen3安装

```shell
sudo apt install libeigen3-dev
```

## 3. 算法环境配置

- 首先应复制准备好的安装包`nx.tar.gz`到~/Downloads文件夹下，然后解压。

  ```shell
  cd ~/Downloads
  tar -zxvf nx.tar.gz
  ```

  ```shell
  cd nx
  ```

### 3.1. ceres安装

- 安装依赖

  ```shell
  sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
  ```

- 编译安装

  ```shell
  cd ~/Downloads/nx/
  unzip ceres-solver-2.1.0.zip
  ```

  ```shell
  cd ceres-solver-2.1.0
  mkdir build
  cd build
  cmake ..
  ```

  ```shell
  make -j8
  ```

  ```shell
  sudo make install
  ```

### 3.2. Opencv安装

- 删除jetpack自带的opencv

  ```shell
  sudo apt-get purge libopencv*
  ```

- 安装依赖

  ```shell
  sudo apt install libgtk2.0-dev pkg-config
  ```

- 编译安装

  ```shell
  cd ~/Downloads/nx/
  unzip opencv-3.4.16.zip
  ```

  ```shell
  cd opencv-3.4.16
  mkdir build
  cd build
  ```

  ```shell
  cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=ON -D CUDA_ARCH_BIN="6.2" -D CUDA_ARCH_PTX="" -D WITH_CUBLAS=ON -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON -D ENABLE_NEON=ON -D WITH_LIBV4L=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D WITH_QT=ON -D WITH_OPENGL=ON -D OPENCV_GENERATE_PKGCONFIG=ON ..
  ```
  
  ```shell
  sudo make -j8
  ```

  ```shell
  sudo make install
  ```

### 3.3. MVSDK安装

```shell
cd ~/Downloads/nx/
unzip MVviewer_Ver2.3.2_Linux_arm_aarch64_Build20220401.zip
```

```shell
sudo chmod 777 MVviewer_Ver2.3.2_Linux_arm_aarch64_Build20220401.run 
sudo ./MVviewer_Ver2.3.2_Linux_arm_aarch64_Build20220401.run
```

- 期间需要输入两次`yes`
- 如果报错类似`5.10.104-tegra`什么文件找不到，可以尝试以下操作

  ```shell
  sudo mv /lib/modules/5.10.104-tegra/ /lib/modules/5.10.104-tegra_old
  sudo ln -s /usr/src/linux-headers-5.10.104-tegra-ubuntu20.04_aarch64/kernel-5.10/ /lib/modules/5.10.104-tegra
  sudo ./MVviewer_Ver2.3.2_Linux_arm_aarch64_Build20220401.run
  ```

- test(插了摄像头的情况下):

  ```shell  
  cd /opt/HuarayTech/MVviewer/bin
  ./run.sh
  ```

### 3.4. QTCreator安装

```shell
sudo apt install qtbase5-dev
sudo apt install qt5-default qtcreator -y
sudo apt install libclang-common-8-dev
```

## 4. 导航环境配置
### 4.1. ROS
建议采用鱼香ROS一键安装
```shell
wget http://fishros.com/install -O fishros && . fishros
```

### 4.2. 安装导航相关的包
 - tf2-msgs:
```shell
 sudo apt-get install ros-noetic-tf2-sensor-msgs 
```
 - livox-ros-driver2:
  参考<https://github.com/Livox-SDK/livox_ros_driver2>
```shell
   git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```
  安装Livox-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```
  复制livox-ros-driver2到工作空间
  修改CMakeLists.txt,删除if-else以及else中的全部内容

```shell
sudo apt-get install ros-noetic-voxel-grid
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-geographic-*
sudo apt-get install ros-noetic-serial
sudo apt-get install ros-noetic-costmap-converter
sudo apt-get install ros-noetic-mbf-costmap*
sudo apt-get install ros-noetic-libg2o
sudo apt-get install ros-noetic-gtsam

```
  删除pointcloud_segmentation中的 INCLUDE_DIRS include
  
  删除pointcloud_segmentation中的 INCLUDE_DIRS include
  删除FAST_LIO_SAM CmakeList.txt geographic依赖中的"REQUIRED"
  

  安装fmt和sophus

### 4.x.GeographicLib
打开包里的GeographicLib，cmake即可  
## 5. 软件安装

### 5.1. VSCode安装

```shell
cd ~/Downloads/nx/
sudo dpkg -i code_1.65.2-1648564670_arm64.deb
```

- 安装完成后，可以在终端输入`code`打开vscode

### 5.2. 向日葵安装

- 由于ubuntu桌面管理环境和向日葵有冲突，需要从默认的gdm3切换到lightdm

  ```
  sudo apt-get update
  sudo apt-get install lightdm   
  ```

- 安装完成，弹出窗口，选择ok-->lightdm-->ok

- 查看当前桌面管理环境

  ```shell
  cat /etc/X11/default-display-manager
  ```

- 安装向日葵

  ```shell
  cd ~/Downloads/nx/sunloginclient-10.0.2.24779_kylin_arm64
  sudo dpkg -i sunloginclient-10.0.2.24779_kylin_arm64.deb  
  ```

- 在`Application`里面就可以找到向日葵了。

- 如果之后出现什么问题可以参考blog：<https://blog.csdn.net/weixin_44942126/article/details/118786584>
### 5.3. 安装中文输入法
- 先安装fcitx：
  ```shell
  sudo apt install fcitx-bin
  sudo apt install fcitx-table
  ```
 - 再卸载ibus
  ```shell
  sudo apt purge ibus
  ```
 打开Language Support，若提醒语言支持未完全安装，点remind me later就好，忽视掉。然后把Key board input method system从ibus改为fcitx。
 - 重启
 - 安装google拼音输入法
  ```shell
  sudo apt-get install fcitx fcitx-googlepinyin -y
  ```
  参考blog:<https://blog.csdn.net/ControlLearner/article/details/121704125>
  
### 5.4. 安装chromium
 ```shell
 sudo snap install chromium
 ```
### 5.5. 安装GithubDesktop
 - 访问:<https://github.com/shiftkey/desktop/releases>，下载对应版本。
 ```shell
 sudo dpkg -i GitHubDesktop-linux-arm64-3.3.12-linux2.deb 
 ```
