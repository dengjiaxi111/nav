#include "relocalization/relocalization.h"
#include <thread>
/*
    slh: 这个节点用于进行重定位
    通过调用service触发
*/

Relocalization::Relocalization(): rclcpp::Node("relocalization")
{
    RCLCPP_INFO(this->get_logger(),"Node Relocalization running!");

    this->declare_parameter<std::string>("pcl_path", "/home/nuc/navigationros2/ros2-humble/src/relocalization/pcd/");
    this->declare_parameter<std::string>("file_name", "scan.pcd");
    this->declare_parameter<std::vector<double>>("lidar_pos",std::vector<double>{0.1, -0.1499, 0.3});
    this->declare_parameter<std::vector<double>>("lidar_rot", std::vector<double>{0.326,0,0,0.94});  //X Y Z W
    this->declare_parameter<std::vector<double>>("init_pos",std::vector<double>{0, 0, 0});
    this->declare_parameter<std::vector<double>>("init_rot", std::vector<double>{0,0,0,1});  //X Y Z W
    this->declare_parameter<bool>("converge_once", true);
    this->declare_parameter<bool>("use_relocalization", true);
    this->declare_parameter<bool>("/auto_relocalization_enabled", false);
    this->declare_parameter<bool>("test_mode", false);
    this->declare_parameter<int>("con_frame", 5);

    this->get_parameter<std::string>("pcl_path", pcd_path_);
    this->get_parameter<std::string>("file_name", file_name_);
    this->get_parameter<std::vector<double>>("lidar_pos", lidar_pos_);
    this->get_parameter<std::vector<double>>("lidar_rot", lidar_rot_);
    this->get_parameter<std::vector<double>>("init_pos", init_pos_);
	this->get_parameter<std::vector<double>>("init_rot", init_rot_);
    this->get_parameter<bool>("converge_once", converge_once_);
    this->get_parameter<bool>("use_relocalization", use_relocalization_);
    this->get_parameter<bool>("/auto_relocalization_enabled", auto_relocalization_enabled_);
    this->get_parameter<bool>("test_mode", test_mode_);
    this->get_parameter<int>("con_frame", max_cloud_num_);

    auto_relocalization_enabled_ = false;
    std::string path = pcd_path_ + file_name_;

    //旧方案，定时器触发重定位
    //relocalization_timer_ = this->create_wall_timer(std::chrono::seconds(20), std::bind(&Relocalization::relocalization_timer_cb, this));
    // 新方案，创建服务，由行为树调用
    relocalization_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "trigger_relocalization",
        std::bind(&Relocalization::handle_relocalization_request, this,
            std::placeholders::_1, std::placeholders::_2));

    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),std::bind(&Relocalization::tf_timer_cb, this));

    lidar_init_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    setInitTF();

    scan_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_registered", 1, std::bind(&Relocalization::pointcloud_cb, this, std::placeholders::_1));
    initpose_sub_  = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 11, std::bind(&Relocalization::initialpose_cb, this, std::placeholders::_1));

    // 读取行为树发布的是否允许重定位的标志位，已废弃
    auto_relocalization_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/auto_relocalization_enabled", 10, std::bind(&Relocalization::auto_relocalization_cb, this, std::placeholders::_1));
            

    priori_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("priori_cloud", 10);
    
    priori_cloud_ = std::make_shared<PointCloudXYZI>();
    input_cloud_  = std::make_shared<PointCloudXYZI>();
    scan_cloud_ = std::make_shared<PointCloudXYZI>();

    // 监听base_link到odom的坐标变换
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    if(use_relocalization_){
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(path, *priori_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "get priori pcd failed, path: %s %s" , path, RESET);
            //throw std::runtime_error("Failed to load PCD file");
        }
        //源点云降采样
        sor_.setInputCloud(priori_cloud_);
        sor_.setLeafSize(0.1f,0.1f,0.1f);
        sor_.filter(*priori_cloud_);

        //用于rviz可视化的点云(高度降采样)
        auto priori_cloud_filtered = std::make_shared<PointCloudXYZI>();

        // 先把点云旋转正
        // Eigen::Affine3f fixed_transform = Eigen::Affine3f::Identity();
        // Eigen::Quaternionf q(lidar_rot_[3],lidar_rot_[0],lidar_rot_[1],lidar_rot_[2]);
        // fixed_transform.rotate(q);
        // fixed_transform.pretranslate(Eigen::Vector3f(lidar_pos_[0] ,lidar_pos_[1] ,lidar_pos_[2]+0.3));
        // pcl::transformPointCloud(*priori_cloud_,*priori_cloud_,fixed_transform);


        sor_.setInputCloud(priori_cloud_);
        sor_.setLeafSize(0.1f,0.1f,0.1f);
        sor_.filter(*priori_cloud_filtered);
        
        RCLCPP_INFO(this->get_logger(), "get priori pcd success!");
        RCLCPP_INFO(this->get_logger(), "point num: %d", priori_cloud_->points.size());
        
        // // 测试用
        // input_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("test_cloud", 10);
        // *input_cloud_ = *priori_cloud_;

        // inputCloudProcess(input_cloud_);

        // cloudRegistration(priori_cloud_, input_cloud_); 

        // pcl::toROSMsg(*input_cloud_,input_cloud_msg_);
        // input_cloud_msg_.header.frame_id = "camera_init";
        //
        pcl::toROSMsg(*priori_cloud_filtered, priori_cloud_msg_);
        priori_cloud_msg_.header.frame_id = "map";
    }
    else{
        priori_cloud_ = nullptr;

    }

    converged_ = false;
    init_pose_get_ = false;
    is_registration_running_ = false;
}

// 由于云台旋转时进行重定位会导致一定误差，故最好不要在此种情况下进行重定位
void Relocalization::auto_relocalization_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    auto_relocalization_enabled_ = msg->data;
}



//前期测试重定位时使用，现已弃用
void Relocalization::inputCloudProcess(PointCloudXYZI::Ptr& cloud)
{ 
    // 进行一些降采样以模拟实际点云帧
    sor_.setInputCloud(cloud);
    sor_.setLeafSize(0.2f,0.2f,0.2f);
    sor_.filter(*cloud);

    pass_.setInputCloud(cloud);
    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(-0.5, 4);
    pass_.setNegative(false);
    pass_.filter(*cloud);

    // 先把点云旋转正，方便接下来进行旋转和平移测试
    Eigen::Affine3f fixed_transform = Eigen::Affine3f::Identity();
    Eigen::Quaternionf q(lidar_rot_[3],lidar_rot_[0],lidar_rot_[1],lidar_rot_[2]);
    fixed_transform.rotate(q);
    fixed_transform.pretranslate(Eigen::Vector3f(lidar_pos_[0] ,lidar_pos_[1] ,lidar_pos_[2]));

    pcl::transformPointCloud(*cloud,*cloud,fixed_transform);
    
    Eigen::Affine3f test_transform = Eigen::Affine3f::Identity();

    // 应用一些平移和旋转变换用于测试
    /* 注意 这里运用的变换将左乘点云坐标，其实是map到odom的变换，即odom到map的逆变换
         简单来讲，如果你想模拟车子在(3,3,0)的情况，就得在这里输入(-3,-3,0) */
    test_transform.rotate(Eigen::AngleAxisf(0.4, Eigen::Vector3f::UnitZ()));
    test_transform.pretranslate(Eigen::Vector3f(-3,-3,0));
    pcl::transformPointCloud(*cloud,*cloud,test_transform);

    // 转换回原坐标
    pcl::transformPointCloud(*cloud,*cloud,fixed_transform.inverse());
}



//点云配准
void Relocalization::cloudRegistration(PointCloudXYZI::Ptr& source, PointCloudXYZI::Ptr& target)
{
    RCLCPP_INFO(this->get_logger(), "Starting registration");
    PointCloudXYZ::Ptr source_xyz = std::make_shared<PointCloudXYZ>();
    PointCloudXYZ::Ptr target_xyz = std::make_shared<PointCloudXYZ>();
    PointCloudXYZ::Ptr output_xyz = std::make_shared<PointCloudXYZ>();

    pcl::copyPointCloud(*source,*source_xyz);
    pcl::copyPointCloud(*target,*target_xyz);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();

    auto start_time = this->get_clock()->now();
    std::cout << odom2init_transform_.translation() << std::endl;
    std::cout << odom2init_transform_.rotation() << std::endl;
    // 将点云从odom变换到map坐标系
    pcl::transformPointCloud(*target_xyz, *target_xyz, odom2init_transform_);

    // 使用small_GICP配准
    // 注意：必须使用release版本编译，否则会有百倍以上性能损失
    gicp_.setNumThreads(8);
    gicp_.setNumNeighborsForCovariance(30);
    gicp_.setMaxCorrespondenceDistance(2.0);
    gicp_.setMaximumIterations(100);
    gicp_.setVoxelResolution(0.1);
    gicp_.setRegistrationType("GICP");  

    // Set input point clouds.
    gicp_.setInputTarget(target_xyz);
    gicp_.setInputSource(source_xyz);
    
    gicp_.align(*output_xyz);
    Eigen::Matrix4f temp_m4 = gicp_.getFinalTransformation();
    Eigen::Affine3f temp(temp_m4);
    trans = temp * trans;
    auto end_time = this->get_clock()->now();

    double time_diff = (end_time-start_time).seconds();
    RCLCPP_INFO(this->get_logger(),(RED + "GICP converge time: %f" + RESET).c_str(), time_diff);

    RCLCPP_INFO(this->get_logger(),"fast_GICP score is %f", gicp_.getFitnessScore());

    RCLCPP_INFO(this->get_logger(),"fast_GICP transformation matrix:");
    std::cout << RED << gicp_.getFinalTransformation() << RESET << std::endl;

    // 计算变换矩阵中的X-Y总位移，若过大则放弃本次配准结果
    float total_translation = temp.translation().norm();
    if(converged_ && total_translation*total_translation - temp.translation()[2]*temp.translation()[2] > 1.0){
        RCLCPP_WARN(this->get_logger(), 
            "Registration result rejected: converged_ translation too large (%.2f m)", 
            total_translation);
        return;
    }

    if (total_translation > 2.0) {
        RCLCPP_WARN(this->get_logger(), 
            "Registration result rejected: translation too large (%.2f m)", 
            total_translation);
        converged_ = false;
        return;
    }
    odom2init_transform_ = trans.inverse() * odom2init_transform_;
    converged_ = true;
}

// 设置重定位初始位姿
void Relocalization::setInitTF(void)
{
    
    //输入的先验点云应为map坐标系下的
    odom2init_tf_.header.frame_id = "map";
    odom2init_tf_.child_frame_id = "odom";
    odom2init_transform_ = Eigen::Affine3f::Identity();
    Eigen::Quaternionf q2(init_rot_[3],init_rot_[0],init_rot_[1],init_rot_[2]);
    odom2init_transform_.rotate(q2);
    odom2init_transform_.pretranslate(Eigen::Vector3f(init_pos_[0], init_pos_[1], init_pos_[2]));
    //odom2init_transform_ = lidar_init2map_trans.inverse() * odom2init_transform_ * lidar_init2map_trans;
    odom2init_tf_.transform.translation.x = odom2init_transform_.translation().x();
    odom2init_tf_.transform.translation.y = odom2init_transform_.translation().y();
    odom2init_tf_.transform.translation.z = odom2init_transform_.translation().z();
    Eigen::Matrix3f rotation_matrix = odom2init_transform_.rotation();
    Eigen::Quaternionf quaternion(rotation_matrix);
    odom2init_tf_.transform.rotation.x = quaternion.x();
    odom2init_tf_.transform.rotation.y = quaternion.y();
    odom2init_tf_.transform.rotation.z = quaternion.z();
    odom2init_tf_.transform.rotation.w = quaternion.w();
}

// 对点云进行处理
void Relocalization::pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
    PointCloudXYZI::Ptr new_cloud(new PointCloudXYZI);
    pcl::fromROSMsg(*cloud, *new_cloud);
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    cloud_buffer_.push_back(new_cloud);

    // 保证最多保存 max_cloud_num_ 帧
    if (cloud_buffer_.size() > max_cloud_num_)
        cloud_buffer_.pop_front();
}

void Relocalization::mergeRecentClouds(PointCloudXYZI::Ptr& merged_cloud)
{
    merged_cloud->clear();
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    for (const auto& cloud : cloud_buffer_)
    {
        *merged_cloud += *cloud;
    }
}


void Relocalization::tf_timer_cb(void)
{
    odom2init_tf_.header.stamp = this->get_clock()->now();

    odom2init_tf_.transform.translation.x = odom2init_transform_.translation().x();
    odom2init_tf_.transform.translation.y = odom2init_transform_.translation().y();
    odom2init_tf_.transform.translation.z = odom2init_transform_.translation().z();
    Eigen::Matrix3f rotation_matrix = odom2init_transform_.rotation();
    //std::cout << "called!" << std::endl;
    Eigen::Quaternionf quaternion(rotation_matrix);
    odom2init_tf_.transform.rotation.x = quaternion.x();
    odom2init_tf_.transform.rotation.y = quaternion.y();
    odom2init_tf_.transform.rotation.z = quaternion.z();
    odom2init_tf_.transform.rotation.w = quaternion.w();
    odom_tf_broadcaster_->sendTransform(odom2init_tf_);
}

void Relocalization::relocalization_timer_cb()
{
    priori_cloud_msg_.header.stamp = this->get_clock()->now();
    priori_cloud_pub_->publish(priori_cloud_msg_);
    
    if (!test_mode_ && !auto_relocalization_enabled_) {
        return;
    }

    if(use_relocalization_ && init_pose_get_){

        if(converge_once_ && converged_){
            return;
        }
        if(!input_cloud_->points.empty() && !is_registration_running_){
            is_registration_running_ = true;
            // 点云非空且上一次配准已经结束，则再次异步执行配准
            std::thread([this]() {
                cloudRegistration(priori_cloud_, input_cloud_);
                is_registration_running_ = false; 
            }).detach();
        }        
    }
}

// 服务回调函数
bool Relocalization::handle_relocalization_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // 在Rviz中显示先验点云
    priori_cloud_msg_.header.stamp = this->get_clock()->now();
    priori_cloud_pub_->publish(priori_cloud_msg_);

    if (!use_relocalization_ || !init_pose_get_) {
        response->success = false;
        response->message = "Relocalization is not available or initial pose not set";
        return true;
    }
    if (converge_once_ && converged_) {
        response->success = false;
        response->message = "Already converged";
        return true;
    }
    if (!cloud_buffer_.empty() && !is_registration_running_) {
        is_registration_running_ = true;
        std::thread([this, response]() {
            PointCloudXYZI::Ptr merged_cloud(new PointCloudXYZI);
            mergeRecentClouds(merged_cloud);
            cloudRegistration(priori_cloud_, merged_cloud);
            is_registration_running_ = false;
            std::cout << "111111" << std::endl;
        }).detach();
        
        response->success = true;
        response->message = "Relocalization started";
    } else {
        response->success = false;
        response->message = "Input cloud empty or registration is running";
    }
    
    return true;
}

// rviz手动指定初始位姿处理
void Relocalization::initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "get init pose from RVIZ");
    /* 
        相当于已知T(base_link->map)， 而T(base_link->map) = T(odom->map) * T(base_link->odom)
        而我们需要的是T(odom->map)
        故有 T(odom->map) = T(base_link->map) * T(base_link->odom)^(-1)
    */
    //std::lock_guard<std::mutex> lock(mtx_);
    tf2::Transform tf_odom2baselink;
    tf2::Transform tf_baselink2map;
    tf2::Transform transform_odom2map;

    // 在Rviz中显示先验点云
    priori_cloud_msg_.header.stamp = this->get_clock()->now();
    priori_cloud_pub_->publish(priori_cloud_msg_);

    
    // 万恶的类型转换,geometry_msgs、tf2、Eigen分别有自己的齐次变换矩阵和四元数等等
    tf_baselink2map.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            0.3));
    tf_baselink2map.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x,
                                                msg->pose.pose.orientation.y,
                                                msg->pose.pose.orientation.z,
                                                msg->pose.pose.orientation.w));

    // 先尝试获取此时的T(odom->base_link),即T(base_link->odom)^(-1)
    geometry_msgs::msg::TransformStamped geotrans_odom2baselink;
    try
    {
        geotrans_odom2baselink = tf_buffer_->lookupTransform("base_link", "odom",tf2::TimePointZero);
    }   
    catch (tf2::TransformException & ex) 
    {
        RCLCPP_WARN(this->get_logger(), "failed to get transform odom to base_link");
        return;
    }

    tf_odom2baselink.setOrigin(tf2::Vector3(geotrans_odom2baselink.transform.translation.x,
                                            geotrans_odom2baselink.transform.translation.y,
                                            geotrans_odom2baselink.transform.translation.z));
    tf_odom2baselink.setRotation(tf2::Quaternion(geotrans_odom2baselink.transform.rotation.x,
                                                geotrans_odom2baselink.transform.rotation.y,
                                                geotrans_odom2baselink.transform.rotation.z,
                                                geotrans_odom2baselink.transform.rotation.w));

    RCLCPP_INFO(this->get_logger(),"x:%f,y:%f ,z:%f ", tf_odom2baselink.getOrigin().x(),tf_odom2baselink.getOrigin().y(), tf_odom2baselink.getOrigin().z());
    double roll,pitch,yaw;
    transform_odom2map = tf_baselink2map * tf_odom2baselink;
    tf2::Quaternion q1 = transform_odom2map.getRotation();
    tf2::Matrix3x3(q1).getRPY(roll,pitch,yaw);
    Eigen::AngleAxisf yaw_angle(float(yaw), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch_angle(float(pitch), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf roll_angle(float(roll), Eigen::Vector3f::UnitX());
    Eigen::Quaternionf q2 = yaw_angle * pitch_angle * roll_angle;

    odom2init_tf_.header.frame_id = "map";
    odom2init_tf_.child_frame_id = "odom";
    odom2init_transform_ = Eigen::Affine3f::Identity();
    
    odom2init_transform_.rotate(q2);
    odom2init_transform_.pretranslate(Eigen::Vector3f(transform_odom2map.getOrigin().getX(),transform_odom2map.getOrigin().getY(),transform_odom2map.getOrigin().getZ()));
    RCLCPP_INFO(this->get_logger(),"TS: x:%f,y:%f ,z:%f ", odom2init_transform_.translation().x(),odom2init_transform_.translation().y(), odom2init_transform_.translation().z());
    
    if(!init_pose_get_){
        init_pose_get_ = true;
    }

    // 若使用重定位，则触发一次
    if(use_relocalization_){
        std::cout << "cloud_buffer_.size:" << cloud_buffer_.size() << std::endl;
        if(cloud_buffer_.size() >= max_cloud_num_ && !is_registration_running_){
            // 点云非空且上一次配准已经结束，则再次异步执行配准
            is_registration_running_ = true;
            std::thread([this]() {
                PointCloudXYZI::Ptr merged_cloud(new PointCloudXYZI);
                mergeRecentClouds(merged_cloud);
                cloudRegistration(priori_cloud_, merged_cloud);
                is_registration_running_ = false;
            }).detach();

        }        
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Relocalization>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);  // 8线程执行器
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


