#ifndef __CHASEENEMY__
#define __CHASEENEMY__
#include <iostream>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robots_msgs/msg/game_status.hpp"
#include "robots_msgs/msg/robot_status.hpp"
#include "robots_msgs/msg/my_path.hpp"
#include "robots_msgs/msg/mode_cmd.hpp"
#include "robots_msgs/msg/enemy_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "common/enemy_info.hpp" 
#include "common/line_obstacle_checker.hpp"

#define FIXED_DISTANCE_STRATEGY 0  // 设置为1启用固定距离策略，设置为0使用原策略

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

using namespace std;
namespace myBT{
class ChaseEnemy:public BT::StatefulActionNode
{
private:

    bool debug_flag_;
    shared_ptr<rclcpp::Node> parent_node_ptr_;
    string yaml_path_;
    string package_path_;

    double chase_radius_;
    double max_costmap_value_;
    bool is_chasing_{false};  // 追踪状态标志

    shared_ptr<robots_msgs::msg::ModeCmd>       modecmd_BB_;
    shared_ptr<robots_msgs::msg::GameStatus>    game_status_BB_;
    shared_ptr<robots_msgs::msg::RobotStatus>   robot_status_BB_;
    shared_ptr<geometry_msgs::msg::TransformStamped> transform_BB_;
    vector<EnemyInfo> enemy_info_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;//可视化的marker指针
    shared_ptr<visualization_msgs::msg::MarkerArray> marker_ptr;
    shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_;
    shared_ptr<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>> costmap_sub_;
    robots_msgs::msg::EnemyPose         enemy_pose_;
    shared_ptr<rclcpp::Subscription<robots_msgs::msg::EnemyPose>> enemy_data_sub_;
    rclcpp::Time    last_get_enemy_time_;

    geometry_msgs::msg::PoseStamped     point_goal_;   
    shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> goal_publisher_;
    rclcpp::Time last_publish_time_;
    rclcpp::Time last_valid_enemy_time_;


    std::vector<std::pair<double, double>> centre_polygon_vertices_;  // 中央高地顶点
    std::vector<std::pair<double, double>> red_half_vertices_;        // 红方半场顶点
    std::vector<std::pair<double, double>> blue_half_vertices_;  // 蓝方半场顶点
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    enum class Region { CENTER, RED_HALF, BLUE_HALF, UNKNOWN };
    Region current_region_{Region::UNKNOWN};

    bool isPointInPolygon(double x, double y, std::vector<std::pair<double, double>> vertices);
    bool isInSameRegionAsEnemy(double ex, double ey, double sx, double sy);
    void visualizePolygon(std::vector<std::pair<double, double>>, int);  // 可视化多边形区域
public:
    ChaseEnemy(const string &name,const BT::NodeConfiguration &config, shared_ptr<rclcpp::Node> parent_node) : BT::StatefulActionNode(name, config)
    {
        parent_node_ptr_ = parent_node;
        package_path_ = ament_index_cpp::get_package_share_directory("mytree");

        marker_ptr = std::make_shared<visualization_msgs::msg::MarkerArray>();
        
        parent_node_ptr_->declare_parameter<bool>("ChaseEnemy_debug",true);

        parent_node_ptr_->get_parameter("ChaseEnemy_debug",debug_flag_);
        parent_node_ptr_->get_parameter_or("footprints_path",yaml_path_,package_path_+"/config/footprints.yaml");

        goal_publisher_ = parent_node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        marker_pub_ = parent_node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>("chase_enemy_markers", 10);

        last_publish_time_ = parent_node_ptr_->get_clock()->now();

        parent_node_ptr_->declare_parameter<double>("chase_radius",1.0);
        parent_node_ptr_->declare_parameter<double>("max_costmap_value",100.0);

        parent_node_ptr_->get_parameter("chase_radius",chase_radius_);
        parent_node_ptr_->get_parameter("max_costmap_value",max_costmap_value_);

        // 声明多边形顶点参数
        parent_node_ptr_->declare_parameter<std::vector<double>>("centrepolygon.x", std::vector<double>());
        parent_node_ptr_->declare_parameter<std::vector<double>>("centrepolygon.y", std::vector<double>());

        parent_node_ptr_->declare_parameter<std::vector<double>>("red_half.x", std::vector<double>());
        parent_node_ptr_->declare_parameter<std::vector<double>>("red_half.y", std::vector<double>());
        parent_node_ptr_->declare_parameter<std::vector<double>>("blue_half.x", std::vector<double>());
        parent_node_ptr_->declare_parameter<std::vector<double>>("blue_half.y", std::vector<double>());

        // 读取各个区域顶点
        std::vector<double> px, py;
        parent_node_ptr_->get_parameter("centrepolygon.x", px);
        parent_node_ptr_->get_parameter("centrepolygon.y", py);
        // 确保x和y坐标数量相同
        if (px.size() != py.size()) {
            RCLCPP_ERROR(parent_node_ptr_->get_logger(), "Polygon vertices x and y coordinates count mismatch!");
            return;
        }
        // 存储多边形顶点
        for (size_t i = 0; i < px.size(); i++) {
            centre_polygon_vertices_.push_back({px[i], py[i]});
        }

        parent_node_ptr_->get_parameter("red_half.x", px);
        parent_node_ptr_->get_parameter("red_half.y", py);
        if (px.size() != py.size()) {
            RCLCPP_ERROR(parent_node_ptr_->get_logger(), "Polygon vertices x and y coordinates count mismatch!");
            return;
        }
        for (size_t i = 0; i < px.size(); i++) {
            red_half_vertices_.push_back({px[i], py[i]});
        }

        parent_node_ptr_->get_parameter("blue_half.x", px);
        parent_node_ptr_->get_parameter("blue_half.y", py);
        if (px.size() != py.size()) {
            RCLCPP_ERROR(parent_node_ptr_->get_logger(), "Polygon vertices x and y coordinates count mismatch!");
            return;
        }
        for (size_t i = 0; i < px.size(); i++) {
            blue_half_vertices_.push_back({px[i], py[i]});
        }

        // 初始化TF相关对象
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(parent_node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
        // 添加订阅器以获取 costmap
        costmap_sub_ = parent_node_ptr_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 2,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                costmap_ = msg;
            });
    }

    void setCurrentPositionAsGoal() {
        point_goal_.header.frame_id = "map";
        point_goal_.header.stamp = parent_node_ptr_->get_clock()->now();
        point_goal_.pose.position.x = transform_BB_->transform.translation.x;
        point_goal_.pose.position.y = transform_BB_->transform.translation.y;
        point_goal_.pose.position.z = transform_BB_->transform.translation.z;
        point_goal_.pose.orientation = transform_BB_->transform.rotation;
        
        goal_publisher_->publish(point_goal_);
        
        // 可视化当前位置为目标点
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header.frame_id = "map";
        goal_marker.header.stamp = parent_node_ptr_->get_clock()->now();
        goal_marker.ns = "chase_enemy";
        goal_marker.id = 1;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.scale.x = 0.25;
        goal_marker.scale.y = 0.25;
        goal_marker.scale.z = 0.25;
        goal_marker.color.a = 1.0;
        goal_marker.color.r = 0.0;
        goal_marker.color.g = 1.0;
        goal_marker.color.b = 0.0;
        goal_marker.pose = point_goal_.pose;
    
        marker_ptr->markers.clear();
        marker_ptr->markers.push_back(goal_marker);
        marker_pub_->publish(*marker_ptr);
    }

    static BT::PortsList providedPorts()
    {
        BT::PortsList ports_list;

        ports_list.insert(BT::BidirectionalPort<std::shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd"));
        ports_list.insert(BT::BidirectionalPort<std::shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus"));
        ports_list.insert(BT::InputPort<std::shared_ptr<nav_msgs::msg::OccupancyGrid>>("costmap"));
        ports_list.insert(BT::InputPort<std::shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus"));
        ports_list.insert(BT::InputPort<vector<EnemyInfo>>("enemypose"));
        ports_list.insert(BT::InputPort<std::shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos"));
        return ports_list;
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

bool ChaseEnemy::isPointInPolygon(double x, double y,std::vector<std::pair<double, double>> vertices) {
    bool inside = false;
    size_t j = vertices.size() - 1;

    for (size_t i = 0; i < vertices.size(); i++) {
        if ((vertices[i].second > y) != (vertices[j].second > y) &&
            (x < (vertices[j].first - vertices[i].first) * (y - vertices[i].second) /
                     (vertices[j].second - vertices[i].second) +
                     vertices[i].first)) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

bool ChaseEnemy::isInSameRegionAsEnemy(double ex, double ey, double sx, double sy) {
    bool eC = isPointInPolygon(ex, ey, centre_polygon_vertices_);
    bool sC = isPointInPolygon(sx, sy, centre_polygon_vertices_);
    if (eC && sC) { current_region_ = Region::CENTER; return true; }

    bool eR = isPointInPolygon(ex, ey, red_half_vertices_);
    bool sR = isPointInPolygon(sx, sy, red_half_vertices_);
    if (eR && sR) { current_region_ = Region::RED_HALF; return true; }

    bool eB = isPointInPolygon(ex, ey, blue_half_vertices_);
    bool sB = isPointInPolygon(sx, sy, blue_half_vertices_);
    if (eB && sB) { current_region_ = Region::BLUE_HALF; return true; }

    current_region_ = Region::UNKNOWN;

    return false;  // 不在同一区域，不追
}

BT::NodeStatus ChaseEnemy::onStart()
{

    cout << GREEN << "-----ChaseEnemy onSTART!----" << RESET << endl;

    getInput<shared_ptr<robots_msgs::msg::GameStatus>>("gamestatus",game_status_BB_);
    getInput<shared_ptr<robots_msgs::msg::RobotStatus>>("robotstatus",robot_status_BB_);
    getInput<vector<EnemyInfo>>("enemypose", enemy_info_);
    getInput<shared_ptr<geometry_msgs::msg::TransformStamped>>("currentpos",transform_BB_);
    getInput<shared_ptr<robots_msgs::msg::ModeCmd>>("modecmd",modecmd_BB_);

    
    if (game_status_BB_ == nullptr || robot_status_BB_ == nullptr || transform_BB_ == nullptr)
    {
        std::cerr << RED << "[ChaseEnemy] Failed to get game status, robot status, transform" << RESET << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    marker_ptr->markers.clear(); 
    visualizePolygon(centre_polygon_vertices_,2);
    visualizePolygon(red_half_vertices_,3);
    visualizePolygon(blue_half_vertices_,4);
    marker_pub_->publish(*marker_ptr);
    
    for (auto& enemy : enemy_info_) {
        if(debug_flag_){
            cout << BLUE << "enemy" <<  enemy._id << "  " <<  "visible: " << enemy._visible << " shootable: " << enemy._shootable << RESET <<  endl;
        }

        if (enemy._visible && enemy._shootable && (enemy._id != 0)) {
            //检查是否在同一区域内
            const auto& sx = transform_BB_->transform.translation.x;
            const auto& sy = transform_BB_->transform.translation.y;
            const auto& ex = enemy._pose.pose.position.x;
            const auto& ey = enemy._pose.pose.position.y;

            if (!isInSameRegionAsEnemy(ex, ey, sx, sy)) {
                if (debug_flag_) {
                    cout << YELLOW << "Enemy not in same region. Skipping chase." << RESET << endl;
                }
                return BT::NodeStatus::FAILURE;
            }
            last_valid_enemy_time_ = parent_node_ptr_->get_clock()->now();
            return BT::NodeStatus::RUNNING;
        }
    }

    if(debug_flag_){
        cout << CYAN << "NO VALID ENEMY TO CHASE. " << RESET << endl;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ChaseEnemy::onRunning()
{
    std::cout << GREEN << "-----ChaseEnemy onRunning!----" << RESET << std::endl;

    getInput<vector<EnemyInfo>>("enemypose", enemy_info_);

    // 设置模式
    modecmd_BB_->should_chassis_spin = 1;
    modecmd_BB_->should_gimbal_patrol = 1;
    modecmd_BB_->use_capacity = 0;
    modecmd_BB_->buy_bullet = 0;
    modecmd_BB_->rebirth = 0;

    EnemyInfo* target_enemy = nullptr;
    for (auto& enemy : enemy_info_) {
        if (enemy._visible && enemy._shootable && (enemy._id != 0)) {
            target_enemy = &enemy;
            last_valid_enemy_time_ = parent_node_ptr_->get_clock()->now();
            cout << enemy._id << endl << endl;
            cout << target_enemy->_pose.pose.position.y << endl << endl;
            break;
        }
    }
    if (!target_enemy) {
        rclcpp::Time now = parent_node_ptr_->get_clock()->now();
        double time_since_last_enemy = (now - last_valid_enemy_time_).seconds();
        const double enemy_wait_timeout = 1.5; 
        
        if (time_since_last_enemy < enemy_wait_timeout) {
            if (debug_flag_) {
                std::cout << YELLOW << "[ChaseEnemy] Enemy lost, waiting for " 
                        << enemy_wait_timeout - time_since_last_enemy << "s..." << RESET << std::endl;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (debug_flag_) {
            std::cout << YELLOW << "[ChaseEnemy] Enemy lost too long, abort chase." << RESET << std::endl;
        }
        return BT::NodeStatus::FAILURE;
    }

    // 获取enemy在map下的坐标
    double ex = 0,ey = 0;
    ex = target_enemy->_pose.pose.position.x;
    ey = target_enemy->_pose.pose.position.y;

    // 检查enemy是否在允许区域内
    if (!isInSameRegionAsEnemy(ex, ey, transform_BB_->transform.translation.x, transform_BB_->transform.translation.y)) {
        if (debug_flag_) {
            std::cout << YELLOW << "Enemy is outside of allowed area: ("
                        << ex << ", " << ey << ")" << RESET << std::endl;
        }
        is_chasing_ = false;  // 重置状态
        return BT::NodeStatus::FAILURE;
    }

    // 只在当前大区内寻找有效点
    const auto &cur_poly = (current_region_ == Region::CENTER
        ? centre_polygon_vertices_
        : (current_region_ == Region::RED_HALF
           ? red_half_vertices_
           : blue_half_vertices_));

    double current_distance = sqrt(
        pow(transform_BB_->transform.translation.x - ex, 2) +
        pow(transform_BB_->transform.translation.y - ey, 2)
    );

    if (debug_flag_) {
        std::cout << BLUE << "Current distance to enemy: " << current_distance  <<  RESET << std::endl;
    }

    // 状态转换逻辑
    // if (!is_chasing_) {
    //     // 如果没在追踪状态，距离大于3.5m才开始追踪
    //     if (current_distance > 3.5) {
    //         is_chasing_ = true;
    //     } else {
    //         // 保持当前位置
    //         setCurrentPositionAsGoal();
    //         return BT::NodeStatus::RUNNING;
    //     }
    // } 
    // else {
    //     // 如果正在追踪，直到接近到1.5m才停止追踪
    //     if (current_distance <= 1.5) {
    //         is_chasing_ = false;
    //         setCurrentPositionAsGoal();
    //         return BT::NodeStatus::RUNNING;
    //     }
    // } 
#if FIXED_DISTANCE_STRATEGY
    // 固定距离策略
    const double TARGET_MIN_DISTANCE = 1.15;
    const double TARGET_MAX_DISTANCE = 1.25;
    
    if (current_distance < TARGET_MIN_DISTANCE || current_distance > TARGET_MAX_DISTANCE) {
        is_chasing_ = true;
        chase_radius_ = 1.2;  // 设置理想追踪距离为1.2米
    } else {
        // 在理想距离范围内，保持当前位置
        setCurrentPositionAsGoal();
        return BT::NodeStatus::RUNNING;
    }
#else
    // 原有策略
    if (!is_chasing_) {
        if (current_distance > 3.5 || current_distance < 1.5) {
            is_chasing_ = true;
        } else {
            setCurrentPositionAsGoal();
            return BT::NodeStatus::RUNNING;
        }
    } 
    else {
        if (current_distance <= 2.5 && current_distance >= 1.5) {
            is_chasing_ = false;
            setCurrentPositionAsGoal();
            return BT::NodeStatus::RUNNING;
        }
    }
#endif

    const auto& info = costmap_->info;
    double resolution = info.resolution;
    double origin_x = info.origin.position.x;
    double origin_y = info.origin.position.y;
    uint32_t width = info.width;
    uint32_t height = info.height;

    std::vector<geometry_msgs::msg::PoseStamped> reachable_edge_points;
    marker_ptr = std::make_shared<visualization_msgs::msg::MarkerArray>();

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = parent_node_ptr_->get_clock()->now();
    edge_marker.ns = "chase_enemy";
    edge_marker.id = 0;
    edge_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.scale.x = 0.15;
    edge_marker.scale.y = 0.15;
    edge_marker.scale.z = 0.15;
    edge_marker.color.a = 1.0;
    edge_marker.color.r = 0.0;
    edge_marker.color.g = 0.0;
    edge_marker.color.b = 1.0;

    double delta_angle = M_PI / 90.0; // 每2度取一个点
    for (double theta = 0; theta < 2 * M_PI; theta += delta_angle) {
        double px = ex + chase_radius_ * cos(theta);
        double py = ey + chase_radius_ * sin(theta);
        
        // 检查index处的目标点是否在多边形内
        if (!isPointInPolygon(px, py, cur_poly)) {
            continue; // 如果点不在多边形内，则跳过
        }

        // 检查目标点能否直视敌方
        bool visible = myBT::isLineObstacleFree(
            std::pair<double,double>{transform_BB_->transform.translation.x, transform_BB_->transform.translation.y},
            std::pair<double,double>{px, py},
            *costmap_, 99);
        
        if(!visible) {
            if (debug_flag_) {
                std::cout << YELLOW << "Point (" << px << ", " << py << ") is blocked by obstacles." << RESET << std::endl;
            }
            continue; // 如果点被障碍物阻挡，则跳过
        }

        int map_x = static_cast<int>((px - origin_x) / resolution);
        int map_y = static_cast<int>((py - origin_y) / resolution);

        if (map_x < 0 || map_y < 0 || map_x >= static_cast<int>(width) || map_y >= static_cast<int>(height))
            continue;

        int index = map_y * width + map_x;
        if (index >= costmap_->data.size()) {
            continue;
        }
        int8_t cost = costmap_->data[index];
        if (cost >= 0 && cost < max_costmap_value_ ) {
            geometry_msgs::msg::PoseStamped pt;
            pt.header.frame_id = "map";
            pt.pose.position.x = px;
            pt.pose.position.y = py;
            pt.pose.position.z = 0;
            pt.pose.orientation.w = 1.0;

            reachable_edge_points.push_back(pt);

            geometry_msgs::msg::Point vis_point;
            vis_point.x = px;
            vis_point.y = py;
            vis_point.z = 0;
            edge_marker.points.push_back(vis_point);
        }
    }

    marker_ptr->markers.push_back(edge_marker);

    if (reachable_edge_points.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    // 找离机器人最近的点
    double min_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::PoseStamped closest_point;

    for (const auto& pt : reachable_edge_points) {
        double dx = pt.pose.position.x - transform_BB_->transform.translation.x;
        double dy = pt.pose.position.y - transform_BB_->transform.translation.y;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_point = pt;
        }
    }

    point_goal_ = closest_point;

    // 添加绿色Marker表示目标点
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = parent_node_ptr_->get_clock()->now();
    goal_marker.ns = "chase_enemy";
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.scale.x = 0.25;
    goal_marker.scale.y = 0.25;
    goal_marker.scale.z = 0.25;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    goal_marker.color.b = 0.0;
    goal_marker.pose = point_goal_.pose;

    marker_ptr->markers.push_back(goal_marker);


    rclcpp::Time now = parent_node_ptr_->get_clock()->now();
    if ((now - last_publish_time_).seconds() > 0.5) {
        goal_publisher_->publish(point_goal_);
        last_publish_time_ = now;

        if (debug_flag_) {
            std::cout << GREEN << "Chase goal published at: ("
                      << point_goal_.pose.position.x << ", "
                      << point_goal_.pose.position.y << ")" << RESET << std::endl;
        }
    }


    marker_pub_->publish(*marker_ptr);


    return BT::NodeStatus::SUCCESS;
}




void ChaseEnemy::onHalted()
{
    is_chasing_ = false;
    current_region_ = Region::UNKNOWN;
    marker_ptr->markers.clear();
}

void ChaseEnemy::visualizePolygon(std::vector<std::pair<double, double>> vertices, int id) {
    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = "map";
    polygon_marker.header.stamp = parent_node_ptr_->get_clock()->now();
    polygon_marker.ns = "chase_enemy";
    polygon_marker.id = id;  // 使用新的ID，避免与其他marker冲突
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.scale.x = 0.1;  // 线宽
    polygon_marker.color.a = 1.0;
    polygon_marker.color.r = 0.26; 
    polygon_marker.color.g = 0.8;
    polygon_marker.color.b = 1.0;

    // 添加多边形的顶点
    for (const auto& vertex : vertices) {
        geometry_msgs::msg::Point p;
        p.x = vertex.first;
        p.y = vertex.second;
        p.z = 0;
        polygon_marker.points.push_back(p);
    }

    // 闭合多边形
    if (!vertices.empty()) {
        geometry_msgs::msg::Point p;
        p.x = vertices[0].first;
        p.y = vertices[0].second;
        p.z = 0;
        polygon_marker.points.push_back(p);
    }
    marker_ptr->markers.push_back(polygon_marker);
}

}

#endif