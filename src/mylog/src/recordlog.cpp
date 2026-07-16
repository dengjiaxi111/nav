#include "rclcpp/rclcpp.hpp"
#include <robots_msgs/msg/game_status.hpp>
#include <robots_msgs/msg/robot_status.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <robots_msgs/msg/mode_cmd.hpp>  
#include <robots_msgs/msg/enemy_pose.hpp>  
#include <tf2_msgs/msg/tf_message.hpp>

#include "nlohmann/json.hpp"
#include <fstream>
#include <vector>
#include <chrono>
#include <string>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <filesystem>
#include <mutex>

using json = nlohmann::json;

class DataLoggerNode : public rclcpp::Node
{
public:
    DataLoggerNode() : Node("data_logger_node")
    {
        game_status_sub_ = this->create_subscription<robots_msgs::msg::GameStatus>(
            "/GameFeedBack", 10, std::bind(&DataLoggerNode::game_Status_Callback, this, std::placeholders::_1));

        robot_status_sub_ = this->create_subscription<robots_msgs::msg::RobotStatus>(
            "/RobotFeedBack", 10, std::bind(&DataLoggerNode::robot_Status_Callback, this, std::placeholders::_1));

        world_coords_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&DataLoggerNode::world_Coords_Callback, this, std::placeholders::_1));

        mode_cmd_sub_ = this->create_subscription<robots_msgs::msg::ModeCmd>(
            "/ModeCmd", 10, std::bind(&DataLoggerNode::mode_Cmd_Callback, this, std::placeholders::_1));

        enemy_pose_sub_ = this->create_subscription<robots_msgs::msg::EnemyPose>(
            "/enemypose", 10, std::bind(&DataLoggerNode::enemy_Pose_Callback, this, std::placeholders::_1));

            using std::filesystem::path;
            using std::filesystem::exists;
            using std::filesystem::create_directory;
        
            // 自动获取当前工作路径并向上查找 "ros2_humble" 目录
            path current_path = std::filesystem::current_path();
            path target_dir;
        
            // 查找父目录中名为 "ros2_humble" 的目录
            while (current_path.has_parent_path()) {
                if (current_path.filename() == "ros2-humble") {
                    target_dir = current_path / "logs";
                    break;
                }
                current_path = current_path.parent_path();
            }
        
            if (target_dir.empty()) {
                RCLCPP_ERROR(this->get_logger(), "未找到 ros2_humble 目录，无法设置日志路径");
                throw std::runtime_error("ros2-humble 目录不存在");
            }
        
            // 确保 "logs" 目录存在
            if (!exists(target_dir)) {
                create_directory(target_dir);
                RCLCPP_INFO(this->get_logger(), "创建日志目录: %s", target_dir.c_str());
            }
        
            // 设置输出文件名并测试文件创建
            std::string timestamp = get_timestamp();
            output_file_ = (target_dir / ("log_" + timestamp + ".json")).string();
            RCLCPP_INFO(this->get_logger(), "日志文件路径: %s", output_file_.c_str());

    
    }

private:
    void game_Status_Callback(const robots_msgs::msg::GameStatus::SharedPtr msg)
    {
        json data;
        data["game_type"] = msg->game_type;
        data["game_progress"] = msg->game_progress;
        data["stage_remain_time"] = msg->stage_remain_time;
        data["my_hp"] = msg->my_hp;
        data["my_outpost_hp"] = msg->my_outpost_hp;
        data["my_base_hp"] = msg->my_base_hp;
        data["enemy_1_robot_hp"] = msg->enemy_1_robot_hp;
        data["enemy_2_robot_hp"] = msg->enemy_2_robot_hp;
        data["enemy_3_robot_hp"] = msg->enemy_3_robot_hp;
        data["enemy_4_robot_hp"] = msg->enemy_4_robot_hp;
        data["enemy_5_robot_hp"] = msg->enemy_5_robot_hp;
        data["enemy_7_robot_hp"] = msg->enemy_7_robot_hp;
        data["enemy_outpost_hp"] = msg->enemy_outpost_hp;
        data["enemy_base_hp"] = msg->enemy_base_hp;
        data["external_supply_area_occupied"] = msg->external_supply_area_occupied;
        data["inner_supply_area_occupied"] = msg->inner_supply_area_occupied;
        data["supply_area_occupied"] = msg->supply_area_occupied;
        data["remaining_gold_coin"] = msg->remaining_gold_coin;
        data["timestamp"] = get_timestamp();

        received_game_data_.push_back(data);
        save_Data_To_json();
    }

    void robot_Status_Callback(const robots_msgs::msg::RobotStatus::SharedPtr msg)
    {
        json data;
        data["robot_id"] = msg->robot_id;
        data["recovery_buff"] = msg->recovery_buff;
        data["defence_buff"] = msg->defence_buff;
        data["vulnerability_buff"] = msg->vulnerability_buff;
        data["attack_buff"] = msg->attack_buff;
        data["remaining_energy"] = msg->remaining_energy;
        data["projectile_allowance_17mm"] = msg->projectile_allowance_17mm;
        data["fort_area"] = msg->fort_area;
        data["supply_blood_area"] = msg->supply_blood_area;
        data["local_exchange_projectile_allowance_17mm"] = msg->local_exchange_projectile_allowance_17mm;
        data["remote_exchange_projectile_times"] = msg->remote_exchange_projectile_times;
        data["remote_exchange_health_times"] = msg->remote_exchange_health_times;
        data["can_free_rebirth"] = msg->can_free_rebirth;
        data["can_instant_rebirth"] = msg->can_instant_rebirth;
        data["rebirth_need_coin"] = msg->rebirth_need_coin;
        data["ttk_status"] = msg->ttk_status;
        data["projectile_allowance_17mm_remain"] = msg->projectile_allowance_17mm_remain;
        data["target_position_x"] = msg->target_position_x;
        data["target_position_y"] = msg->target_position_y;
        data["timestamp"] = get_timestamp();

        received_robot_data_.push_back(data);
    }

    void world_Coords_Callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto &transform : msg->transforms)
        {
            // 检查是否是 baselink 到 map 的变换
            if (transform.header.frame_id == "map" && transform.child_frame_id == "baselink")
            {
                std::string timestamp = get_timestamp();
    
                json world_coords_data;
                world_coords_data["world_x"] = transform.transform.translation.x;
                world_coords_data["world_y"] = transform.transform.translation.y;
                world_coords_data["world_z"] = transform.transform.translation.z;
                world_coords_data["rotation_x"] = transform.transform.rotation.x;
                world_coords_data["rotation_y"] = transform.transform.rotation.y;
                world_coords_data["rotation_z"] = transform.transform.rotation.z;
                world_coords_data["rotation_w"] = transform.transform.rotation.w;
                world_coords_data["timestamp"] = timestamp;
    
                received_world_coords_data_.push_back(world_coords_data);
    
                // 保存数据到 JSON 文件
                save_Data_To_json();
            }
        }
    }

    void mode_Cmd_Callback(const robots_msgs::msg::ModeCmd::SharedPtr msg)
    {
        json data;
        data["should_chassis_spin"] = msg->should_chassis_spin;
        data["should_gimbal_patrol"] = msg->should_gimbal_patrol;
        data["timestamp"] = get_timestamp();

        received_mode_cmd_data_.push_back(data);
    }

    void enemy_Pose_Callback(const robots_msgs::msg::EnemyPose::SharedPtr msg)
    {
        json data;
        data["enemy_dist"] = msg->enemy_dist;
        data["enemy_ori"] = msg->enemy_ori;
        data["enemy_num"] = msg->enemy_num;
        data["enemy_x"] = msg->enemy_x;
        data["enemy_y"] = msg->enemy_y;
        data["timestamp"] = get_timestamp();

        received_enemy_pose_data_.push_back(data);
    }

    void save_Data_To_json()
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        if (output_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "输出文件路径为空！");
            return;
        }

        if (received_game_data_.empty() || received_robot_data_.empty() ||
            received_world_coords_data_.empty() || received_mode_cmd_data_.empty()) {
            RCLCPP_WARN(this->get_logger(), "数据容器为空或大小不一致，跳过保存！");
            return;
        }

        size_t min_size = std::min({received_game_data_.size(), received_robot_data_.size(),
                                    received_world_coords_data_.size(), received_mode_cmd_data_.size(),
                                    received_enemy_pose_data_.size()});

        json all_logged_data;
        for (size_t i = 0; i < min_size; ++i)
        {
            json combined_data;
            combined_data["game_data"] = received_game_data_[i];
            combined_data["robot_data"] = received_robot_data_[i];
            combined_data["world_coords_data"] = received_world_coords_data_[i];
            combined_data["mode_cmd_data"] = received_mode_cmd_data_[i];
            combined_data["enemy_pose_data"] = received_enemy_pose_data_[i];
            all_logged_data.push_back(combined_data);
        }

        std::ofstream output_file(output_file_, std::ios::app);
        if (!output_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件: %s", output_file_.c_str());
            return;
        }

        output_file << std::setw(4) << all_logged_data << std::endl;

        // 清理缓存
        received_game_data_.clear();
        received_robot_data_.clear();
        received_world_coords_data_.clear();
        received_mode_cmd_data_.clear();
        received_enemy_pose_data_.clear();
    }

    std::string get_timestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%dT%H:%M:%S");
        return ss.str();
    }

    // Data storage
    std::vector<json> received_game_data_;
    std::vector<json> received_robot_data_;
    std::vector<json> received_world_coords_data_;
    std::vector<json> received_mode_cmd_data_;
    std::vector<json> received_enemy_pose_data_;

    std::string output_file_;
    std::mutex data_mutex;

    rclcpp::Subscription<robots_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
    rclcpp::Subscription<robots_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr world_coords_sub_;
    rclcpp::Subscription<robots_msgs::msg::ModeCmd>::SharedPtr mode_cmd_sub_;
    rclcpp::Subscription<robots_msgs::msg::EnemyPose>::SharedPtr enemy_pose_sub_;
};

int main(int argc, char **argv)
{
    std::cout << "Start record log" << std::endl;

    // 初始化 ROS 2 系统
    rclcpp::init(argc, argv);

    // 检查 ROS 2 是否正常初始化
    if (rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROS 2 system initialized successfully.");
        // 创建并启动节点
        rclcpp::spin(std::make_shared<DataLoggerNode>());
    } else {
        // 初始化失败，打印错误信息并退出
        std::cerr << "ROS 2 initialization failed!" << std::endl;
    }

    // 关闭 ROS 2 系统
    rclcpp::shutdown();
    return 0;
}

