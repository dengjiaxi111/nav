#ifndef __MODE_CONTROL_NODE__
#define __MODE_CONTROL_NODE__

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robots_msgs/msg/mode_cmd.hpp"
#include "robots_msgs/msg/game_status.hpp"
#include "robots_msgs/msg/robot_status.hpp"
#include "std_msgs/msg/bool.hpp"

#include "common/is_point_in_area.hpp"

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

using namespace std;

namespace myBT
{
    class ModeControl: public BT::SyncActionNode
    {
    private:
        shared_ptr<rclcpp::Node> parent_node_ptr_;
        bool debug_flag_;

        // 盲道区域
        std::vector<std::pair<double,double>>  red_blindway_zone_;
        std::vector<std::pair<double,double>> blue_blindway_zone_;

        // 控制量
        bool in_buff_mode_;
        bool in_relocalization_mode_;
        shared_ptr<robots_msgs::msg::ModeCmd> modecmd_BB_; 
        robots_msgs::msg::ModeCmd modecmd_;
        rclcpp::Publisher<robots_msgs::msg::ModeCmd>::SharedPtr modecmd_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_relocalization_pub_;

        // 状态数据
        shared_ptr<robots_msgs::msg::GameStatus> game_status_BB_;
        shared_ptr<robots_msgs::msg::RobotStatus> robot_status_BB_;
        shared_ptr<geometry_msgs::msg::TransformStamped> currentpos_;

        rclcpp::Time last_damage_time_;
        int last_time_hp_;
        bool last_in_relocalization_mode_ = false;
        int patrol_mode_backup_ = 1; // 保留为 int，避免 bool 限制合法值
        
    public:
        ModeControl(const string& name, 
                       const BT::NodeConfiguration& config,
                       shared_ptr<rclcpp::Node> parent_node) 
            : BT::SyncActionNode(name, config)
        {
            parent_node_ptr_ = parent_node;
            
            std::vector<double> red_blindway_zone_x;
            std::vector<double> red_blindway_zone_y;
            std::vector<double> blue_blindway_zone_x;
            std::vector<double> blue_blindway_zone_y;

            // 参数初始化   
            parent_node_ptr_->declare_parameter<bool>("ModeControl_debug", true);
            parent_node_ptr_->get_parameter("ModeControl_debug", debug_flag_);

            parent_node_ptr_->declare_parameter<std::vector<double>>("red_blindway.x",std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("red_blindway.y",std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("blue_blindway.x",std::vector<double>());
            parent_node_ptr_->declare_parameter<std::vector<double>>("blue_blindway.y",std::vector<double>());

            parent_node_ptr_->get_parameter("red_blindway.x", red_blindway_zone_x);
            parent_node_ptr_->get_parameter("red_blindway.y", red_blindway_zone_y);
            parent_node_ptr_->get_parameter("blue_blindway.x", blue_blindway_zone_x);
            parent_node_ptr_->get_parameter("blue_blindway.y", blue_blindway_zone_y);

            // 确保x和y坐标数量相同
            if (red_blindway_zone_x.size() != red_blindway_zone_y.size()) {
                RCLCPP_ERROR(parent_node_ptr_->get_logger(), "Polygon vertices x and y coordinates count mismatch!");
                return;
            }
            if (blue_blindway_zone_x.size() != blue_blindway_zone_y.size()) {
                RCLCPP_ERROR(parent_node_ptr_->get_logger(), "Polygon vertices x and y coordinates count mismatch!");
                return;
            }
            
            // 存储多边形顶点
            for (size_t i = 0; i < red_blindway_zone_x.size(); i++) {
                red_blindway_zone_.push_back({red_blindway_zone_x[i], red_blindway_zone_y[i]});
            }
            for (size_t i = 0; i < blue_blindway_zone_x.size(); i++) {
                blue_blindway_zone_.push_back({blue_blindway_zone_x[i], blue_blindway_zone_y[i]});
            }

            // Publisher初始化
            modecmd_publisher_ = parent_node_ptr_->create_publisher<robots_msgs::msg::ModeCmd>("/ModeCmd", 10);

            // 变量初始化
            in_buff_mode_ = false;
            in_relocalization_mode_ = false;
            modecmd_BB_ = make_shared<robots_msgs::msg::ModeCmd>();
            modecmd_BB_->rebirth = 1;
            modecmd_ = *modecmd_BB_;
            
            last_time_hp_ = 400;
            last_damage_time_ = parent_node_ptr_->get_clock()->now();
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("in_buff_mode"),
                BT::InputPort<bool>("in_relocalization_mode"),
                BT::InputPort<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"),
                BT::InputPort<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"),
                BT::InputPort<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos"),
                BT::BidirectionalPort<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd")
            };
        }

        BT::NodeStatus tick() override;

    private:
        void checkChassisSpin();
        void updateRelocalizationMode();
        void updateBuffMode();
        void publishModeCmd();
    };
    
    BT::NodeStatus ModeControl::tick()
    {
        cout << GREEN << "---ModeControl tick---" << RESET << endl;

        // 获取输入
        getInput<bool>("in_buff_mode", in_buff_mode_);
        getInput<bool>("in_relocalization_mode", in_relocalization_mode_);
        getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus", game_status_BB_);
        getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus", robot_status_BB_);
        getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd", modecmd_BB_);
        getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos", currentpos_);

        if(!modecmd_BB_ || !game_status_BB_ || !robot_status_BB_ || !currentpos_) {
            RCLCPP_ERROR(parent_node_ptr_->get_logger(), "ModeControl: Missing required input data.");
            return BT::NodeStatus::FAILURE;
        }
        modecmd_ = *modecmd_BB_;

        // 更新各种模式
        checkChassisSpin();
        //updateRelocalizationMode();
        updateBuffMode();
        
        // 发布命令
        publishModeCmd();
        
        // 更新黑板
        *modecmd_BB_ = modecmd_;
        setOutput("modecmd", modecmd_BB_);

        return BT::NodeStatus::FAILURE;
    }

    void ModeControl::checkChassisSpin(){
        bool in_blindway = isPointInArea(currentpos_->transform.translation.x,currentpos_->transform.translation.y,red_blindway_zone_) \
                    || isPointInArea(currentpos_->transform.translation.x,currentpos_->transform.translation.y,blue_blindway_zone_);

        if(modecmd_.should_chassis_spin == 1 ){
            // 脱战反馈
            if(last_time_hp_ - game_status_BB_->my_hp >= 4) {
                last_damage_time_ = parent_node_ptr_->get_clock()->now();
            }

            if((parent_node_ptr_->get_clock()->now() - last_damage_time_).seconds() > 5.0){
                modecmd_.should_chassis_spin = 2;
            }
            else{
                modecmd_.should_chassis_spin = 1;
            }
        }
        last_time_hp_ = game_status_BB_->my_hp;
        
        //盲道减少陀螺
        if(in_blindway){
            std::cout << "in_blindway!" << std::endl << std::endl; 
            if(modecmd_.should_chassis_spin == 2){
                modecmd_.should_chassis_spin = 0;
            }
            else if(modecmd_.should_chassis_spin == 1){
                modecmd_.should_chassis_spin = 1;
            }
        }
    }

    void ModeControl::updateRelocalizationMode(){
        if (!last_in_relocalization_mode_ && in_relocalization_mode_) {
            patrol_mode_backup_ = modecmd_.should_gimbal_patrol; // 只备份一次
            modecmd_.should_gimbal_patrol = 0;
            modecmd_.rebirth = 1;
            modecmd_.buy_bullet = 0;
        }
    
        // 模式切换：true → false，刚刚退出重定位模式
        if (last_in_relocalization_mode_ && !in_relocalization_mode_) {
            modecmd_.should_gimbal_patrol = patrol_mode_backup_;
        }
    
        // 更新上一次的状态
        last_in_relocalization_mode_ = in_relocalization_mode_;
    }

    void ModeControl::updateBuffMode(){
        return;
    }

    void ModeControl::publishModeCmd(){
        if(debug_flag_){
            cout << BLUE << "should_chassis_spin: " << (int)modecmd_.should_chassis_spin << RESET << endl
            << BLUE << "should_gimbal_patrol: " << (int)modecmd_.should_gimbal_patrol << RESET << endl
            << BLUE << "buy_bullet: " << (int)modecmd_.buy_bullet << RESET << endl
            << BLUE << "rebirth: " << (int)modecmd_.rebirth << RESET << endl
            << BLUE << "use_capacity: " << (int)modecmd_.use_capacity << RESET << endl;
        }
        modecmd_publisher_->publish(modecmd_);
    }
}
#endif


    

