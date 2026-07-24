#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <termios.h>
#include <thread>
#include <vector>
#include <deque>
#include <functional>
#include <chrono>
#include <array>

#include "myprotocol.hpp"
#include <tf2/utils.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "decision_messages/msg/our_robot_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/async.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "play_music/async_audio_player.hpp"

using namespace std;

using boost::asio::io_context;
using boost::asio::serial_port;
using boost::asio::ip::tcp;
namespace asio = boost::asio;

namespace rm
{

class SerialNode : public rclcpp::Node
{
public:
    SerialNode()
    : Node("serial_node"),
      io_(),
      port_(io_),
      strand_(asio::make_strand(io_)),
      timer_(io_, chrono::milliseconds(20)),  // 发送频率
      running_(true),
      info_pub_(true),
      enemy_priority_(0b11111110)
    {
        cout << "WholeGetFrame: " << WHOLE_GET_LEN << endl;
        cout << "WholeSendFrame: " << WHOLE_SEND_LEN << endl;

        this->declare_parameter<bool>("on_court", false);
        this->declare_parameter<bool>("debug_flag", false);
        this->declare_parameter<bool>("info_pub",true);
        this->declare_parameter<bool>("enable_batch_read", false);
        this->declare_parameter<bool>("enable_improved_framing", false);
        this->declare_parameter<int>("batch_read_size", 128);
        this->declare_parameter<string>("log_path", "/home/super259/nav/log");

        this->get_parameter("on_court",on_court_);
        this->get_parameter("debug_flag",debug_flag_);
        this->get_parameter("info_pub", info_pub_);
        this->get_parameter("enable_batch_read", enable_batch_read_);
        this->get_parameter("enable_improved_framing", enable_improved_framing_);
        this->get_parameter("batch_read_size", batch_read_size_);
        this->get_parameter("log_path",log_path_);
        RCLCPP_INFO(this->get_logger(),"on_court: %d", static_cast<int>(on_court_));
        RCLCPP_INFO(this->get_logger(),"enable_batch_read: %d (size=%d)",
            static_cast<int>(enable_batch_read_), batch_read_size_);
        RCLCPP_INFO(this->get_logger(),"enable_improved_framing: %d",
            static_cast<int>(enable_improved_framing_));

        // 这个函数返回的是： <install space>/share/your_package_name
        package_path_ = ament_index_cpp::get_package_share_directory("myserial");
        music_package_path_ = ament_index_cpp::get_package_share_directory("play_music");

        chassis_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_gimbal", 5, std::bind(&SerialNode::chas_cmd_callback, this, std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("mypath", 5, std::bind(&SerialNode::path_callback, this, std::placeholders::_1));
        mode_sub_ = this->create_subscription<robots_msgs::msg::ModeCmd>("ModeCmd", 5, std::bind(&SerialNode::modecmd_callback, this, std::placeholders::_1));
        priority_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/enemy_priority", 10,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                enemy_priority_ = msg->data;
            });
        buff_yaw_diff_sub_ = this->create_subscription<std_msgs::msg::Float64>("/yaw_diff", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                _send_frame_._buff_yaw_diff_angle = int((msg->data) * 100);
            });

        // Publisher初始化
        game_pub_ = this->create_publisher<robots_msgs::msg::GameStatus>("GameFeedBack", 2);     // 发布比赛数据话题
        robot_pub_ = this->create_publisher<robots_msgs::msg::RobotStatus>("RobotFeedBack", 2);  // 发布比赛数据话题
        chassis_odom_pub_ = this->create_publisher<robots_msgs::msg::ChassisOdom>("ChassisOdom", 2);  // 发布电控里程计数据
        enemypose_pub_ = this->create_publisher<robots_msgs::msg::EnemyPose>("EnemyPose", 2);   // 发布自瞄敌人信息
        decision_enemy_pub_ = this->create_publisher<decision_messages::msg::EnemyRobotState>("/decision_messages/EnemyRobotState", 10);
        decision_game_pub_ = this->create_publisher<decision_messages::msg::GameState>("/decision_messages/GameState", 10);
        decision_our_pub_ = this->create_publisher<decision_messages::msg::OurRobotState>("/decision_messages/OurRobotState", 10);
        enemy_tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        sentry_control_sub_ = this->create_subscription<sentry_decision::msg::SentryControl>(
            "/sentry/control", 10, std::bind(&SerialNode::sentry_control_callback, this, std::placeholders::_1));
        last_cmd_vel_time_ = this->get_clock()->now();

        // 初始化logger
        logger_init();

        signal(SIGINT, [](int) {
            rclcpp::shutdown();
        });
        
        while(true)
        {
            if(open_failed_cnt_ % 20 == 19){
                RCLCPP_ERROR(this->get_logger(), "串口打开失败次数: %d", open_failed_cnt_);
                AsyncAudioPlayer player(music_package_path_ + "/music/Windows_shutdown.wav");
                this_thread::sleep_for(chrono::milliseconds(1000));
                return;
            }
            try {
                //https://blog.csdn.net/m0_51152048/article/details/141321116 使用udev rules 固定名称
                port_.open("/dev/mystm32");
                set_serial_raw_mode();
                port_.set_option(serial_port::baud_rate(460800)); // 波特率
                port_.set_option(serial_port::character_size(8));   // 数据位
                port_.set_option(serial_port::parity(serial_port::parity::none)); //无校验
                port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));  //一个停止位
                port_.set_option(serial_port::flow_control(serial_port::flow_control::none));   // 禁用流控
            } catch (const boost::system::system_error& e) {
                RCLCPP_ERROR(this->get_logger(), "串口打开失败: %s", e.what());
                this_thread::sleep_for(chrono::milliseconds(500));
                open_failed_cnt_++;
                continue;
            }
            RCLCPP_INFO(this->get_logger(),"打开串口成功！");
            player_ = std::make_shared<AsyncAudioPlayer>(music_package_path_ + "/music/Windows_logon.wav");
            open_failed_cnt_ = 0;
            break;
        }

        callback_ = bind(&SerialNode::msg_callback,this,placeholders::_1);

        // 读串口
        read_loop();

        // 写串口
        start_write_timer();

        io_thread_ = thread([this]() { io_.run(); });

        // 监测线程
        monitor_thread_ = thread(&SerialNode::monitor, this);
    }

    ~SerialNode() override {
        running_ = false;
        port_.close();
        io_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
        if(monitor_thread_.joinable()){
            monitor_thread_.join();
        }
        spdlog::shutdown();
    }


private:
    void logger_init();
    void set_serial_raw_mode();

    void read_loop();
    void read_callback(const boost::system::error_code& ec, size_t);
    void read_batch_callback(const boost::system::error_code& ec, size_t bytes_read);
    void parse_buffer();
    void msg_callback(const WholeGetFrame& msg);
    void write_timer_callback(const boost::system::error_code& ec);
    void start_write_timer();
    void send_msg();
    void monitor();

    void chas_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void modecmd_callback(const robots_msgs::msg::ModeCmd::SharedPtr msg);
    void sentry_control_callback(const sentry_decision::msg::SentryControl::SharedPtr msg);
    void publish_decision_messages(const WholeGetFrame& msg);

    bool debug_flag_;

    // logger相关
    shared_ptr<spdlog::async_logger> my_logger_;
    string package_path_;
    string log_path_;

    // 串口相关
    thread monitor_thread_;  // 用于监控发送失败次数的线程
    io_context io_;
    serial_port port_;
    asio::strand<asio::io_context::executor_type> strand_;
    thread io_thread_;
    function<void(const WholeGetFrame&)> callback_;
    deque<uint8_t> buffer_;
    uint8_t read_byte_ = 0;
    std::array<uint8_t, 256> read_buffer_;
    boost::asio::steady_timer timer_;
    bool running_ = false;
    uint32_t send_count_ = 0;
    const int MAX_BUFFER_SIZE = 1024;
    int snd_failed_cnt_ = 0;
    int open_failed_cnt_ = 0;


    uint8_t enemy_priority_;
    WholeGetFrame _get_frame_;
    WholeSendFrame _send_frame_;

    std::shared_ptr<AsyncAudioPlayer> player_;  
    std::shared_ptr<AsyncAudioPlayer> prepare_player_;
    uint8_t last_game_process_ = 0;
    uint8_t last_game_type_ = 0;
    string music_package_path_;

    // ROS收发 
    bool info_pub_;
    std::shared_ptr<rclcpp::Publisher<robots_msgs::msg::GameStatus>>    game_pub_;
    std::shared_ptr<rclcpp::Publisher<robots_msgs::msg::RobotStatus>>   robot_pub_;
    std::shared_ptr<rclcpp::Publisher<robots_msgs::msg::ChassisOdom>>   chassis_odom_pub_;
    std::shared_ptr<rclcpp::Publisher<robots_msgs::msg::EnemyPose>>     enemypose_pub_;
    std::shared_ptr<rclcpp::Publisher<decision_messages::msg::EnemyRobotState>> decision_enemy_pub_;
    std::shared_ptr<rclcpp::Publisher<decision_messages::msg::GameState>> decision_game_pub_;
    std::shared_ptr<rclcpp::Publisher<decision_messages::msg::OurRobotState>> decision_our_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> enemy_tf_pub_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr priority_sub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> chassis_sub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>> path_sub_;
    std::shared_ptr<rclcpp::Subscription<robots_msgs::msg::ModeCmd>> mode_sub_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>>    buff_yaw_diff_sub_;
    std::shared_ptr<rclcpp::Subscription<sentry_decision::msg::SentryControl>> sentry_control_sub_;

    robots_msgs::msg::GameStatus game_status_;
    robots_msgs::msg::RobotStatus robot_status_;
    robots_msgs::msg::ChassisOdom chassis_odom_speed_;
    robots_msgs::msg::EnemyPose enemypose_;
    geometry_msgs::msg::TransformStamped enemypose_tf_;

    uint16_t chas_cmd_cbk_cnt_ = 0;
    uint16_t path_cbk_cnt_ = 0;
    uint16_t mode_cbk_cnt_ = 0;
    uint16_t send_frame_cnt_ = 0;
    uint16_t get_frame_cnt_ = 0;
    bool on_court_ = false;

    rclcpp::Time last_cmd_vel_time_;

    // 打符用
    double buff_yaw_diff_;

    bool enable_batch_read_;
    bool enable_improved_framing_;
    int batch_read_size_;
};

}
