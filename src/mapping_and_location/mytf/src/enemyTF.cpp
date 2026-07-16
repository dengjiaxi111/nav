#include "mytf/enemyTF.hpp"

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<EnemyTF>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

void EnemyTF::timer_callback()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!enemy_info_._pose_queue.empty()) {
        auto latest_pose = enemy_info_._pose_queue.back();
        send_tf(latest_pose.pose.position.x, latest_pose.pose.position.y);
    }
}

void EnemyTF::enemy_callback(robots_msgs::msg::EnemyPose::SharedPtr ptr)
{
    std::lock_guard<std::mutex> lock(mtx_);

    // !WARNING! 此处的转换需要与视觉组再次确认
    auto current_time = this->get_clock()->now();
    auto time_diff = (current_time - last_enemy_time_).seconds();
    if((ptr->enemy_x != 0) && (time_diff >= 0.05)){
        geometry_msgs::msg::PoseStamped enemy_pose;
        enemy_pose.header.frame_id = "gimbal_link";
        enemy_pose.header.stamp    = this->get_clock()->now();

        enemy_pose.pose.position.x = ptr->enemy_x;
        enemy_pose.pose.position.y = ptr->enemy_y;
        enemy_pose.pose.position.z = 0;
        enemy_pose.pose.orientation.x = 0;
        enemy_pose.pose.orientation.y = 0;
        enemy_pose.pose.orientation.z = 0;
        enemy_pose.pose.orientation.w = 1;

        if (enemy_info_._pose_queue.size() >= STORAGE_LEN) 
        {
                enemy_info_._pose_queue.pop();
        }
        enemy_info_._pose_queue.push(enemy_pose);
        RCLCPP_DEBUG(this->get_logger(),"Valid enemy info get!");
        last_enemy_time_ = current_time;
    }

    
}

void EnemyTF::send_tf(float x,float y)
{

    geometry_msgs::msg::TransformStamped tfs;
    tfs.header.frame_id = "gimbal_link";
    tfs.header.stamp = this->get_clock()->now();
    tfs.child_frame_id = "enemy";
    tfs.transform.translation.x = x;
    tfs.transform.translation.y = y;
    tfs.transform.translation.z = 0.0;

    tfs.transform.rotation.x = 0;
    tfs.transform.rotation.y = 0;
    tfs.transform.rotation.z = 0;
    tfs.transform.rotation.w = 1;

    auto current_time = this->get_clock()->now();
    auto time_diff = (current_time - last_enemy_time_).seconds();
    if(time_diff <= 1)
    {
        broadcaster_->sendTransform(tfs);
    }
}

//TODO: 远期考虑用KF将点云聚类与视觉信息融合
void EnemyTF::predict_enemy_pose(float enemy_x_list[], float enemy_y_list[], int& valid_data_num)
{
    if (valid_data_num == 0){}
    else if (valid_data_num == 1)
    {
        enemy_x_list[1] = enemy_x_list[0];
        enemy_y_list[1] = enemy_y_list[0];
        enemy_x_list[0] += 0.01;
        valid_data_num += 1; 
    }
    else
    {
        float weights[] = {4, 3, 2, 1};
        float weight_nomalized = 0;
        for (int i = 0; i <= valid_data_num - 2; i++)
        {
            weight_nomalized += weights[i];
        } 
        for (int i = 0; i <= valid_data_num - 2; i++)
        {
            weights[i] = weights[i] / weight_nomalized;
        }

        float x_temp = 0, y_temp = 0;
        for (int i = 0; i <= valid_data_num - 2; i++)
        {
            x_temp += (enemy_x_list[i] - enemy_x_list[i + 1]) * weights[i];
            y_temp += (enemy_y_list[i] - enemy_y_list[i + 1]) * weights[i];  
        } 
        for (int i = 0; i <= 3; i++)
        {
            enemy_x_list[i + 1] = enemy_x_list[i];
            enemy_y_list[i + 1] = enemy_y_list[i];
        }
        enemy_x_list[0] = enemy_x_list[1] + x_temp;
        enemy_y_list[0] = enemy_y_list[1] + y_temp;
        if (valid_data_num < 5) valid_data_num ++;
        else valid_data_num =  5;
    } 
}