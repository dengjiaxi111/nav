#include<iostream>
#include "myserial/serial_node.hpp"

namespace rm
{

// 初始化logger
void SerialNode::logger_init()
{   
    // 获取当前时间作为文件名
    auto t = std::time(nullptr);
    std::tm tm;
    localtime_r(&t, &tm);  // 线程安全版本

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");

    spdlog::init_thread_pool(8192, 1);
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path_ +"/serial_log_" + oss.str() + ".txt");

    // 创建异步日志记录器
    my_logger_ = std::make_shared<spdlog::async_logger>(
        "seirial_logger", file_sink, spdlog::thread_pool(),spdlog::async_overflow_policy::block);
    
    // 设置全局的 logger
    spdlog::set_default_logger(my_logger_);
    my_logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    spdlog::flush_on(spdlog::level::info);  // 设置日志刷新条件（每次INFO级别及以上的日志都
}


// 注册一个事件监听器，监听串口的数据到来事件；一旦触发，就执行绑定的回调，同时继续监听下一次数据。
void SerialNode::read_loop()
{
    asio::async_read(port_, asio::buffer(&read_byte_, 1),
            asio::bind_executor(strand_,
                bind(&SerialNode::read_callback, this,
                          placeholders::_1, placeholders::_2)));
}

// 读取字节的回调
void SerialNode::read_callback(const boost::system::error_code& ec, size_t)
{
    if (!ec && running_) {
        buffer_.push_back(read_byte_);
        parse_buffer();
        read_loop();
    }
}

// 拼帧
void SerialNode::parse_buffer()
{
    if(buffer_.size() > MAX_BUFFER_SIZE){
        buffer_.clear();
        RCLCPP_WARN(this->get_logger(),"缓冲区溢出.......清空数据");
    }

    while (buffer_.size() >= WHOLE_GET_LEN) 
    {
        while(!buffer_.empty() && buffer_[0] != HEADER){
            buffer_.pop_front();
        }

        if (buffer_.size() < WHOLE_GET_LEN) return;

        if (buffer_[WHOLE_GET_LEN - 1] != TAIL) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + WHOLE_GET_LEN);
            continue;
        }

        
        copy(buffer_.begin(), buffer_.begin() + WHOLE_GET_LEN, reinterpret_cast<uint8_t*>(&_get_frame_));

        buffer_.erase(buffer_.begin(), buffer_.begin()+WHOLE_GET_LEN);

        if (callback_) {
            my_logger_->info("[RX] {}", _get_frame_.to_string());
            if(debug_flag_){
                _get_frame_.print();
                RCLCPP_INFO(this->get_logger(), "成功接收一帧数据");
                RCLCPP_INFO(this->get_logger(), "当前时间：%.6f 秒", this->now().seconds());
            }
            callback_(_get_frame_);
        }
    }
}

// 在ROS中发布收到的消息
void SerialNode::msg_callback(const WholeGetFrame& msg)
{
    // ------------------ GameStatus ------------------
    game_status_.game_type = msg._game_type;
    game_status_.game_progress = msg._game_process;
    game_status_.stage_remain_time = msg._stage_remain_time;

    game_status_.my_hp = msg._my_HP;
    game_status_.my_outpost_hp = msg._my_outpost_HP;
    game_status_.my_base_hp = msg._my_base_HP;

    game_status_.enemy_1_robot_hp = msg._enemy_1_robot_HP;
    game_status_.enemy_2_robot_hp = msg._enemy_2_robot_HP;
    game_status_.enemy_3_robot_hp = msg._enemy_3_robot_HP;
    game_status_.enemy_4_robot_hp = msg._enemy_4_robot_HP;
    game_status_.enemy_7_robot_hp = msg._enemy_7_robot_HP;
    game_status_.enemy_outpost_hp = msg._enemy_outpost_HP;
    game_status_.enemy_base_hp = msg._enemy_base_HP;

    game_status_.remaining_gold_coin = msg._remaining_gold_coin;

    // 事件数据解析
    game_status_.external_supply_area_occupied = (msg._event_data) & 0x01;
    game_status_.inner_supply_area_occupied = (msg._event_data >> 1) & 0x01;
    game_status_.supply_area_occupied = (msg._event_data >> 2) & 0x01;
    game_status_.our_small_buff_activated = (msg._event_data >> 3) & 0x01;
    game_status_.our_big_buff_activated = (msg._event_data >> 4) & 0x01;
    game_status_.our_fortress_status = (msg._event_data >> 23) & 0x03;
    if(info_pub_){
        game_pub_->publish(game_status_);
    }

    // ------------------ RobotStatus ------------------
    robot_status_.robot_id = msg._robot_id;
    robot_status_.x = msg._x;
    robot_status_.y = msg._y;
    robot_status_.angle = msg._angle;

    robot_status_.recovery_buff = msg._recovery_buff;
    robot_status_.defence_buff = msg._defence_buff;
    robot_status_.vulnerability_buff = msg._vulnerability_buff;
    robot_status_.attack_buff = msg._attack_buff;

    robot_status_.remaining_energy = msg._remaining_energy;
    robot_status_.projectile_allowance_17mm = msg._projectile_allowance_17mm;

    // RFID 相关状态
    robot_status_.fort_area = (msg._rfid_status >> 17) & 0x01;
    robot_status_.supply_blood_area = (msg._rfid_status >> 19) & 0x01;

    // 哨兵兑换信息
    robot_status_.local_exchange_projectile_allowance_17mm = msg._sentry_info & 0x07FF;
    robot_status_.remote_exchange_projectile_times = (msg._sentry_info >> 11) & 0x0F;
    robot_status_.remote_exchange_health_times = (msg._sentry_info >> 15) & 0x0F;
    robot_status_.can_free_rebirth = (msg._sentry_info >> 19) & 0x01;
    robot_status_.can_instant_rebirth = (msg._sentry_info >> 20) & 0x01;
    robot_status_.rebirth_need_coin = (msg._sentry_info >> 21) & 0x03FF;


    robot_status_.ttk_status = msg._sentry_info_2 & 0x01;
    robot_status_.projectile_allowance_17mm_remain = (msg._sentry_info_2 >> 1) & 0x07FF;

    robot_status_.target_position_x = msg._target_position_x;
    robot_status_.target_position_y = msg._target_position_y;

    if(info_pub_){
        robot_pub_->publish(robot_status_);
    }

    // ------------------ ChassisOdom ------------------
    chassis_odom_speed_.speed_x = static_cast<float>(msg._speed_x) / 50;
    chassis_odom_speed_.speed_y = static_cast<float>(msg._speed_y) / 50;
    chassis_odom_speed_.speed_w = static_cast<float>(msg._speed_w) / 50;
    chassis_odom_speed_.gimbal_angle = msg._base_yaw;

    chassis_odom_pub_->publish(chassis_odom_speed_);

    // ----------------- 自瞄发送的敌人位置信息 -----------------
    enemypose_.enemy_num    = msg._enemy_id;

    // 从imu系转换到当前的云台系
    enemypose_.enemy_x      = cos(msg._base_yaw /180 * M_PI) * msg._enemy_x - sin(msg._base_yaw /180 * M_PI) * msg._enemy_y;
    enemypose_.enemy_y     = sin(msg._base_yaw /180 * M_PI) * msg._enemy_x + cos(msg._base_yaw /180 * M_PI) * msg._enemy_y;
    if(info_pub_){
        enemypose_pub_->publish(enemypose_);
    }
    
    last_game_process_ = msg._game_process;
    last_game_type_ = msg._game_type;

}

// 开启发送定时
void SerialNode::start_write_timer()
{
    timer_.expires_after(chrono::milliseconds(20));
    timer_.async_wait(asio::bind_executor(strand_,
        bind(&SerialNode::write_timer_callback, this, placeholders::_1)));
}

void SerialNode::write_timer_callback(const boost::system::error_code& ec)
{   
    start_write_timer();
    if (!ec && running_) {
        
        send_msg();
    }
}

void SerialNode::send_msg()
{
        vector<uint8_t> packet(WHOLE_SEND_LEN);
        // 默认见谁打谁
        _send_frame_._priority = this->enemy_priority_;
        auto cmd_vel_time_diff = last_cmd_vel_time_ - this->get_clock()->now();
        if(cmd_vel_time_diff.seconds() >= 0.2){
            // 认为已超时
            _send_frame_._speed_x = 0;
            _send_frame_._speed_y = 0;
        }
        

        memcpy(packet.data(), &_send_frame_, WHOLE_SEND_LEN);

        my_logger_->info("[TX] {}", _send_frame_.to_string());
        if(debug_flag_){
            _send_frame_.print();
        }
        asio::async_write(port_, asio::buffer(packet),
            asio::bind_executor(strand_,
                [this](const boost::system::error_code& ec, size_t) {
                    if (ec) {
                        // TODO: 添加错误处理日志
                        snd_failed_cnt_++;
                    }
                }));
        send_frame_cnt_++;
}

void SerialNode::chas_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    _send_frame_.setAutoDriveMode(true);
    chas_cmd_cbk_cnt_++;
    last_cmd_vel_time_ = this->get_clock()->now();

    if (on_court_){
        if (_get_frame_._game_process == 4){
            _send_frame_._speed_x = msg->linear.x;
            _send_frame_._speed_y = msg->linear.y;
            _send_frame_._speed_w = msg->angular.z;
        }
        else{
            _send_frame_._speed_x = 0;
            _send_frame_._speed_y = 0;
            _send_frame_._speed_w = 0;
        }
    }
    else{
        _send_frame_._speed_x = msg->linear.x;
        _send_frame_._speed_y = msg->linear.y;
        _send_frame_._speed_w = msg->angular.z;
    }

}

void SerialNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // TODO 订阅全局路径并转换为通讯帧中的形式
    path_cbk_cnt_++;

}

void SerialNode::modecmd_callback(const robots_msgs::msg::ModeCmd::SharedPtr msg)
{
    mode_cbk_cnt_++;
    _send_frame_.setAutoDriveMode(true);
    _send_frame_.setChassisMode(msg->should_chassis_spin);
    _send_frame_.setGimbalMode(msg->should_gimbal_patrol);
    _send_frame_.setBuyBullet(msg->buy_bullet);
    _send_frame_.setRebirth(msg->rebirth);
    _send_frame_.setHiPower(msg->use_capacity);
}

void SerialNode::monitor(){
    while (running_) {
        if (snd_failed_cnt_ >= 50) {
            RCLCPP_WARN(this->get_logger(), "串口发送失败次数超过阈值(%d次)，检查串口连接！", snd_failed_cnt_);
            
            // 播放警告音
            AsyncAudioPlayer player(music_package_path_ + "/music/Windows_shutdown.wav");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            snd_failed_cnt_ = 0; 

            // 尝试重启串口
            try {
                // 停止 IO，关闭串口
                io_.post([this]() {
                    boost::system::error_code ec;
                    port_.cancel(ec);
                    port_.close(ec);
                });
                buffer_.clear();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // 重新打开串口
                io_.post([this]() {
                    try {
                        port_.open("/dev/mystm32");
                        port_.set_option(serial_port::baud_rate(460800));
                        port_.set_option(serial_port::character_size(8));
                        port_.set_option(serial_port::parity(serial_port::parity::none));
                        port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
                        port_.set_option(serial_port::flow_control(serial_port::flow_control::none));
                        RCLCPP_INFO(this->get_logger(), "串口重启成功！");
                        snd_failed_cnt_ = 0;
                        player_ = std::make_shared<AsyncAudioPlayer>(music_package_path_ + "/music/Windows_logon.wav");
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        read_loop();  // 重新启动异步接收
                    } catch (const boost::system::system_error &e) {
                        RCLCPP_ERROR(this->get_logger(), "串口重启失败: %s", e.what());
                    }
                });

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "monitor 中串口重启异常: %s", e.what());
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(2)); 
    }
}

}