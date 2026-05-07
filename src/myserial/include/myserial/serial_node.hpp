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
#include <random>

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

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/async.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "play_music/async_audio_player.hpp"

// 决策系统消息（decision_messages）
#include "decision_messages/msg/our_robot_state.hpp"
#include "decision_messages/msg/enemy_robot_state.hpp"
#include "decision_messages/msg/game_state.hpp"
#include "sentry_decision/msg/sentry_control.hpp"
#include "robots_msgs/msg/leg_length.hpp"


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
        this->declare_parameter<bool>("enable_rtt_measure", false);  // RTT 测量开关
        
        // 性能优化开关（用于对比测试）
        this->declare_parameter<bool>("enable_batch_read", false);     // 批量读取优化
        this->declare_parameter<bool>("enable_improved_framing", false); // 改进拼帧逻辑
        // USB CDC Full-Speed: 64B/包, 建议设置为 64 的整数倍 (128=2包, 192=3包)
        this->declare_parameter<int>("batch_read_size", 128);          // 批量读取缓冲区大小
        
        this->declare_parameter<string>("log_path", "/home/nuc/logs");

        this->get_parameter("on_court",on_court_);
        this->get_parameter("debug_flag",debug_flag_);
        this->get_parameter("info_pub", info_pub_);
        this->get_parameter("enable_rtt_measure", enable_rtt_measure_);
        this->get_parameter("enable_batch_read", enable_batch_read_);
        this->get_parameter("enable_improved_framing", enable_improved_framing_);
        this->get_parameter("batch_read_size", batch_read_size_);
        this->get_parameter("log_path",log_path_);
        
        RCLCPP_INFO(this->get_logger(),"on_court: %d", static_cast<int>(on_court_));
        RCLCPP_INFO(this->get_logger(),"enable_rtt_measure: %d", static_cast<int>(enable_rtt_measure_));
        RCLCPP_INFO(this->get_logger(),"enable_batch_read: %d (size=%d)", 
            static_cast<int>(enable_batch_read_), batch_read_size_);
        RCLCPP_INFO(this->get_logger(),"enable_improved_framing: %d", 
            static_cast<int>(enable_improved_framing_));

        // 这个函数返回的是： <install space>/share/your_package_name
        package_path_ = ament_index_cpp::get_package_share_directory("myserial");
        music_package_path_ = ament_index_cpp::get_package_share_directory("play_music");

        chassis_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 5, std::bind(&SerialNode::chas_cmd_callback, this, std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("mypath", 5, std::bind(&SerialNode::path_callback, this, std::placeholders::_1));
        mode_sub_ = this->create_subscription<robots_msgs::msg::ModeCmd>("ModeCmd", 5, std::bind(&SerialNode::modecmd_callback, this, std::placeholders::_1));
        sentry_control_sub_ = this->create_subscription<sentry_decision::msg::SentryControl>(
            "/sentry/control", 10, std::bind(&SerialNode::sentry_control_callback, this, std::placeholders::_1));
        priority_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/enemy_priority", 10,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                enemy_priority_ = msg->data;
            });
        stair_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>("stair_mode", 10,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                _send_frame_._stair_mode = msg->data;
            });
        buff_yaw_diff_sub_ = this->create_subscription<std_msgs::msg::Float64>("/yaw_diff", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                _send_frame_._buff_yaw_diff_angle = int((msg->data) * 100);
            });

        // Publisher初始化
        game_pub_ = this->create_publisher<robots_msgs::msg::GameStatus>("GameFeedBack", 2);     // 发布比赛数据话题
        robot_pub_ = this->create_publisher<robots_msgs::msg::RobotStatus>("RobotFeedBack", 2);  // 发布比赛数据话题
        chassis_odom_pub_ = this->create_publisher<robots_msgs::msg::ChassisOdom>("ChassisOdom", 2);  // 发布电控里程计数据
        leg_length_pub_ = this->create_publisher<robots_msgs::msg::LegLength>("LegLength", 2);   // 发布腿长信息
        enemypose_pub_ = this->create_publisher<robots_msgs::msg::EnemyPose>("EnemyPose", 2);   // 发布自瞄敌人信息
        // 决策系统消息（整合后直接发布，供决策节点订阅）
        our_state_pub_   = this->create_publisher<decision_messages::msg::OurRobotState>("/decision_messages/OurRobotState", 10);
        enemy_state_pub_ = this->create_publisher<decision_messages::msg::EnemyRobotState>("/decision_messages/EnemyRobotState", 10);
        game_state_pub_  = this->create_publisher<decision_messages::msg::GameState>("/decision_messages/GameState", 10);

        enemy_tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        last_cmd_vel_time_ = this->get_clock()->now();

        // 初始化logger
        logger_init();

        // 初始化 RTT 相关
        last_rtt_report_time_ = std::chrono::steady_clock::now();

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
                
                // 获取原生句柄并设置为 RAW 模式
                int fd = port_.native_handle();
                struct termios tty;
                if (tcgetattr(fd, &tty) != 0) {
                    RCLCPP_ERROR(this->get_logger(), "tcgetattr error");
                } else {
                    cfmakeraw(&tty);
                    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
                        RCLCPP_ERROR(this->get_logger(), "tcsetattr error");
                    }
                }

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

            

            const std::array<std::string, 3> music_files = {"曼波.mp3", "欧耶.mp3", "wow.mp3"};
            static thread_local std::mt19937 rng{std::random_device{}()};
            std::uniform_int_distribution<size_t> dist(0, music_files.size()-1 );
            player_ = std::make_shared<AsyncAudioPlayer>(
                music_package_path_ + "/music/" + music_files[dist(rng)]);
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

    void read_loop();
    void read_callback(const boost::system::error_code& ec, size_t);
    void read_batch_callback(const boost::system::error_code& ec, size_t bytes_read);  // 批量读取回调
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
    std::shared_ptr<rclcpp::Publisher<robots_msgs::msg::LegLength>>     leg_length_pub_;
    std::shared_ptr<rclcpp::Publisher<robots_msgs::msg::EnemyPose>>     enemypose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> enemy_tf_pub_;
    // 决策系统消息发布者
    rclcpp::Publisher<decision_messages::msg::OurRobotState>::SharedPtr   our_state_pub_;
    rclcpp::Publisher<decision_messages::msg::EnemyRobotState>::SharedPtr enemy_state_pub_;
    rclcpp::Publisher<decision_messages::msg::GameState>::SharedPtr       game_state_pub_;
    // 决策消息缓存（跨回调聚合：GameStatus 先收到，RobotStatus 后收到，合并后发布）
    decision_messages::msg::OurRobotState   our_state_;
    decision_messages::msg::EnemyRobotState enemy_state_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr priority_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr stair_mode_sub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> chassis_sub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>> path_sub_;
    std::shared_ptr<rclcpp::Subscription<robots_msgs::msg::ModeCmd>> mode_sub_;
    std::shared_ptr<rclcpp::Subscription<sentry_decision::msg::SentryControl>> sentry_control_sub_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>>    buff_yaw_diff_sub_;

    robots_msgs::msg::GameStatus game_status_;
    robots_msgs::msg::RobotStatus robot_status_;
    robots_msgs::msg::ChassisOdom chassis_odom_speed_;
    robots_msgs::msg::LegLength leg_length_msg_;
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

    // RTT 测量相关
    bool enable_rtt_measure_;           // 是否启用 RTT 测量
    
    // RTT 统计
    double rtt_sum_ = 0.0;              // RTT 累计（毫秒）
    double rtt_min_ = 999999.0;         // 最小 RTT（毫秒）
    double rtt_max_ = 0.0;              // 最大 RTT（毫秒）
    uint32_t rtt_count_ = 0;            // 收到的响应数量
    uint32_t rtt_send_count_ = 0;       // 发送的总数量
    std::chrono::steady_clock::time_point last_rtt_report_time_;  // 上次报告时间
    
    // 优化开关
    bool enable_batch_read_;            // 是否启用批量读取
    bool enable_improved_framing_;      // 是否启用改进的帧解析
    int batch_read_size_;               // 批量读取缓冲区大小
    std::array<uint8_t, 256> read_buffer_;  // 批量读取缓冲区
};

}
