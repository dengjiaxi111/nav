#ifndef BACK_TO_FREESPACE_
#define BACK_TO_FREESPACE_

#include <string>
#include <vector>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace my_nav2_behaviors
{
/**
 * @class my_nav2_behaviors::BackToFreeSpace
 * @brief 卡在墙壁、致命障碍物中后直线运动到最近的非障碍物节点
 */
class BackToFreeSpace: public nav2_behaviors::DriveOnHeading<nav2_msgs::action::DriveOnHeading>
{
public:
    BackToFreeSpace() = default;

    void onConfigure() override;

    /**
     * @brief Cleanup server on lifecycle transition
     */
    void onCleanup() override;

    /**
     * @brief Initialization to run behavior
     * @param command Goal to execute
     * @return Status of behavior
     */
    nav2_behaviors::Status onRun(const std::shared_ptr<const nav2_msgs::action::DriveOnHeading::Goal> command) override;

    /**
     * @brief Loop function to run behavior
     * @return Status of behavior
     */
    nav2_behaviors::Status onCycleUpdate() override;

protected:
    /**
   * @brief 寻找可行点
   * @return 可行点集合
   */
    std::vector<geometry_msgs::msg::Point> gatherFreePoints(
        const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius);

    /**
   * @brief 计算安全度
   * @return 代价值
   */
    float calculateSafety(
        const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float angle,
        float radius);

    /**
     * @brief 寻找最佳方向
     * @return 最佳方向
    */
    float findBestDirection(
        const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
        float end_angle, float radius, float angle_increment);

     /**
   * @brief 可视化
   * @return void
   */
    void visualize(const geometry_msgs::msg::Point & target_point);

    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
        marker_pub_;

    
    // parameters
    std::string costmap_service_name_;
    double twist_x_, twist_y_;  //速度
    double search_radius_, robot_radius_; //搜索距离
    int free_threshold_;        //阈值
    bool visualize_;            
};

}
#endif