在Ubuntu 24.04系统上为Livox MID360配置PTP时间同步，核心逻辑是**将你的NUC15作为PTP的主时钟（Grandmaster）**，MID360作为从时钟（Slave）。

根据你提供的网卡信息，连接雷达的网卡是 **`enp86s0`** (IP: 192.168.1.50)。以下是完整的、开机自启的配置教程。

---

### 第一步：检查网卡硬件时间戳支持
虽然软件时间戳也能工作，但硬件时间戳精度最高。NUC15的网卡通常支持硬件PTP。
打开终端，运行：
```bash
sudo ethtool -T enp86s0
```
**查看输出：** 如果在 `Capabilities` 中看到了 `SOF_TIMESTAMPING_TX_HARDWARE` 和 `SOF_TIMESTAMPING_RX_HARDWARE`，说明完美支持硬件时间戳。

### 第二步：安装 PTP 软件
我们需要安装 `linuxptp` 包，它包含了 `ptp4l`（处理PTP协议）和 `phc2sys`（同步系统时间到网卡硬件时间）。
```bash
sudo apt update
sudo apt install linuxptp ethtool -y
```

### 第三步：配置 ptp4l 作为主时钟 (Master)
我们需要创建一个专门的配置文件，强制NUC的这块网卡作为PTP主时钟。

1. 创建并编辑配置文件：
```bash
sudo nano /etc/linuxptp/ptp-master.conf
```
2. 将以下内容粘贴进去（按 `Ctrl+O` 保存，`Enter` 确认，`Ctrl+X` 退出）：
```ini
[global]
# 优先级设置得较小（默认128），确保工控机成为Grandmaster
priority1               127
priority2               127
domainNumber            0
# 使用硬件时间戳 (如果在第一步发现不支持硬件，请将此处改为 software)
time_stamping           hardware

# 允许从系统时钟同步
clock_servo             linreg
delay_mechanism         E2E

# 关键新增：强制使用 PTPv2.0 (IEEE 1588-2008) 而不是 2.1
ptp_minor_version       0

[enp86s0]
# 绑定到你的雷达网卡
```

### 第四步：创建 Systemd 守护进程（开机自启）
为了让PTP在后台稳定运行且开机自启，我们重写两个系统服务。

#### 1. 配置 `ptp4l` 服务
```bash
sudo nano /etc/systemd/system/livox-ptp4l.service
```
粘贴以下内容：
```ini
[Unit]
Description=Livox PTP4L Master Service
After=network.target

[Service]
Type=simple
# 运行ptp4l，使用刚才的配置文件
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp-master.conf -m
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

#### 2. 配置 `phc2sys` 服务
`phc2sys` 的作用是将NUC的系统时间（OS时钟）实时同步到 `enp86s0` 网卡的硬件时钟上，这样 `ptp4l` 广播出去的时间才是准确的系统时间。
```bash
sudo nano /etc/systemd/system/livox-phc2sys.service
```
粘贴以下内容：
```ini
[Unit]
Description=Livox PHC2SYS Service
After=livox-ptp4l.service

[Service]
Type=simple
# -s CLOCK_REALTIME: 以系统时间为主
# -c enp86s0: 将时间同步给网卡硬件
# -w: 等待 ptp4l 启动
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c enp86s0 -w -m -O 0
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 第五步：启动并验证服务

1. **重新加载系统服务并设置开机自启：**
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now livox-ptp4l.service
sudo systemctl enable --now livox-phc2sys.service
```

2. **检查 `ptp4l` 运行状态：**
```bash
sudo journalctl -u livox-ptp4l.service -f
```
*正常情况下，你会看到类似 `assuming the grand master role` 的日志，按 `Ctrl+C` 退出。*

3. **检查 `phc2sys` 运行状态：**
```bash
sudo journalctl -u livox-phc2sys.service -f
```
*你会看到类似 `CLOCK_REALTIME phc offset...` 的日志，说明系统时间和网卡时间正在同步。*

### 第六步：验证 Livox MID360 的时间同步状态

你有两种方式可以验证雷达是否成功吃到了PTP时间：

**方法 A：通过 Livox Viewer 2 (推荐)**
1. 打开 Livox Viewer 2 软件并连接你的 MID360。
2. 点击雷达设置（齿轮图标）。
3. 查看 **Time Sync (时间同步)** 状态，如果显示为 **PTP** 且颜色正常，说明同步成功。（刚启动时可能显示为无或内部时钟，大约需要等待10-30秒才会切换到PTP）。

**方法 B：通过 ROS 驱动验证**
如果你在使用 `livox_ros_driver2`，启动节点后查看终端输出或使用 `ros2 topic echo` 查看点云的话题。
检查点云的 `header.stamp`（时间戳）是否与当前系统时间（`date +%s`）一致。如果不一致，说明使用的是雷达开机以来的相对时间；如果一致，说明PTP配置成功。

---

### 💡 附加重要建议：保持NUC系统时间准确
因为我们将NUC的系统时间（`CLOCK_REALTIME`）作为了整个系统的基准时间，**NUC自身的时间必须准确**。
你的 `wlo1` (10.102.145.175) 连着网，建议确保 Ubuntu 自带的 NTP 同步处于开启状态：
```bash
timedatectl set-ntp true
timedatectl status
```
确认 `System clock synchronized: yes`。这样整个系统的链路就是：
**互联网真实时间 (NTP) -> NUC系统时间 -> NUC网卡时间 (phc2sys) -> Livox MID360 (ptp4l)。**