#ifndef __UPDATEINFO__
#define __UPDATEINFO__
#include <iostream>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robots_msgs/msg/game_status.hpp"
#include "robots_msgs/msg/robot_status.hpp"
#include "robots_msgs/msg/my_path.hpp"
#include "robots_msgs/msg/mode_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/bool.hpp" 



#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

/*
场地图（抽象版）
														  (280,150)dm
		Y	:...::,:::::::.....::.   ..,  . .:,:..:.::::..;;: 
		|	r  .rT177yyzrrccccT7nccc;rc@;;TCULLLrczrCZZZ1TLrDy
		|	i;;;r;Tcccr,rCbbOW$WLT1r7@@@@@@@@,.,rTczEnrZZr$B8,
		|	7g@@C:;;r;;C@@@@@@0rrr,;@$$@@@@@@@@r;rLczr;cr,BW$.
		|	@@UrWG2r@@@@@@7:;;,,U@@@G8$@@@$@@@D;rrL;,@@C$$@;
		|	@@@@@@@@EWG2b@@@:;L;;@@@@@@@n:GO;2@@@gz1n1L@@@@@@$
		|	@@;.y@@S7EK1n@@; ::z@@@j,;n@@z:;c;;@@@bOWOn@@@;.z$
	红	|	@    @@$nEEnW@@, ..c@@@,   C@@@;.. L@@2KUE2@@2   T		蓝
		|	@@;.1@@GiOWE0@@@;rc;,8@gy;:G@@@r.. ;@$T2EKy@@@;:1@
		|	@@@@@@@8cKC7Z@@@@r;Oz:N@@@@@@B;;L,:@@@jKNCU@@@@@@C
		|	;n@$Db@b,rrr;r@@@@8@@@gGN@@@7,;;;:G@@@@@2rWGWr@@@ 
		|	,@$N:ir;LTLL;;1@@@@@@@@$$N;;rrr@@@@@@@r,rr;;,@@@c
		|	,;@88;bzLOnLcT;:.W@@@@@@@@rryrc8DOSUSz;;rrLrrrr:,;
		|	TiB;L;777ZT;yr;rrrr;;.,$;:,rrLzrr;;;;;rrLccccc,  .
		  (0,0) ---------------------------------------------->X
*/

using namespace std;


namespace myBT
{
    class UpdateInfo: public BT::SyncActionNode
    {
    private:

    // debug模式
    bool debug_flag_;
    shared_ptr<rclcpp::Node> parent_node_ptr_;
    string yaml_path_;
    string package_path_;

    robots_msgs::msg::MyPath nav_path_;

    rclcpp::Publisher<robots_msgs::msg::MyPath>::SharedPtr mypath_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; //可视化的marker指针

    // 从电控获取的数据
    shared_ptr<robots_msgs::msg::GameStatus>   game_status_BB_;
    shared_ptr<robots_msgs::msg::RobotStatus> robot_status_BB_;
    rclcpp::Subscription<robots_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
    rclcpp::Subscription<robots_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;

    // 路径点数据
    nav_msgs::msg::Path path_from_planner_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planner_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    // nav2中的位姿
    shared_ptr<tf2_ros::Buffer> tf_buffer_;
    shared_ptr<tf2_ros::TransformListener> tf_listener_;
    shared_ptr<geometry_msgs::msg::TransformStamped> transform_BB_;

    rclcpp::Time last_damage_time_;  // 上次受伤时间
    int last_time_hp_;

    public:
        UpdateInfo(const string &name,const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) : BT::SyncActionNode(name, config)
        {
            parent_node_ptr_ = parent_node;
            package_path_ = ament_index_cpp::get_package_share_directory("mytree");
            
            parent_node_ptr_->declare_parameter<bool>("UpdateInfo_debug",true);
            
            parent_node_ptr_->get_parameter("UpdateInfo_debug",debug_flag_);
            parent_node_ptr_->get_parameter_or("footprints_path",yaml_path_,package_path_+"/config/footprints.yaml");
            transform_BB_ = std::make_shared<geometry_msgs::msg::TransformStamped>();   //初始化指针     


            tf_buffer_ = make_shared<tf2_ros::Buffer>(parent_node->get_clock());
            tf_listener_ = make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            rclcpp::sleep_for(std::chrono::milliseconds(200)); 

            // 进行一波初始化
            
            mypath_publisher_ = parent_node_ptr_->create_publisher<robots_msgs::msg::MyPath>("/mypath",10);

            {       
                game_status_BB_ = make_shared<robots_msgs::msg::GameStatus>();
                robot_status_BB_ = make_shared<robots_msgs::msg::RobotStatus>();

                game_status_sub_ = parent_node_ptr_->create_subscription<robots_msgs::msg::GameStatus>("/GameFeedBack",10,bind(&UpdateInfo::gameCallback,this,placeholders::_1));
                robot_status_sub_ = parent_node_ptr_->create_subscription<robots_msgs::msg::RobotStatus>("/RobotFeedBack",10,bind(&UpdateInfo::robotCallback,this,placeholders::_1));
                
                
            }

            planner_path_sub_ = parent_node_ptr_->create_subscription<nav_msgs::msg::Path>("/plan",1,bind(&UpdateInfo::plannerPathCallback,this,placeholders::_1));

            last_time_hp_ = 400;
            last_damage_time_ = parent_node_ptr_->get_clock()->now();

        }

        static BT::PortsList providedPorts()
        {
            BT::PortsList ports_list;
            ports_list.insert(BT::OutputPort<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"));
            ports_list.insert(BT::OutputPort<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"));
            ports_list.insert(BT::OutputPort<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos"));

            return ports_list;
        }        


        BT::NodeStatus tick() override;

        void pathInfoPub();

        void gameCallback(const robots_msgs::msg::GameStatus::SharedPtr msg);
        void robotCallback(const robots_msgs::msg::RobotStatus::SharedPtr msg);
        void plannerPathCallback(const nav_msgs::msg::Path::SharedPtr msg);


    };

    BT::NodeStatus UpdateInfo::tick()
    {
        cout << GREEN << "---UpdateInfo tick---" << RESET << endl;

        rclcpp::spin_some(parent_node_ptr_);
        
        auto now = parent_node_ptr_->get_clock()->now();
        
        try {
            *transform_BB_  = tf_buffer_->lookupTransform("map", "base_link",tf2::timeFromSec(0),  tf2::durationFromSec(0.1));
        } 
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(parent_node_ptr_->get_logger(), "TRANSFORM LOOKUP FAILED:  %s", ex.what());
        }

        if(debug_flag_)
        {
            cout << BLUE << "game type: " << int(game_status_BB_->game_type) << RESET << endl;
            cout << BLUE << "game progress: " << int(game_status_BB_->game_progress) << RESET << endl;
            cout << BLUE << "stage remain time: " << game_status_BB_->stage_remain_time << RESET << endl;
            cout << BLUE << "robot ID: " << int(robot_status_BB_->robot_id) << RESET << endl;
            cout << BLUE << "sentry hp: " << game_status_BB_->my_hp << RESET << endl;
            if(transform_BB_){
                cout << BLUE << "robot position: x: " << transform_BB_->transform.translation.x 
                    << " y: " << transform_BB_->transform.translation.y << RESET << endl;
            }
        }
        pathInfoPub();

        setOutput("gamestatus",game_status_BB_);
        setOutput("robotstatus",robot_status_BB_);
        setOutput("currentpos",transform_BB_);

        // 由于是fallback控制节点，所以直接返回FAILURE
        return BT::NodeStatus::FAILURE;
    }

    void UpdateInfo::pathInfoPub()
    {
        // TODO: 在小地图上显示烧饼导航轨迹

    }

    void UpdateInfo::gameCallback(const robots_msgs::msg::GameStatus::SharedPtr msg)
    {
        *(this->game_status_BB_) = *msg;
    }

    void UpdateInfo::robotCallback(const robots_msgs::msg::RobotStatus::SharedPtr msg)
    {
        *(this->robot_status_BB_) = *msg;
    }

    void UpdateInfo::plannerPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        //TODO: 
    }

}



#endif
