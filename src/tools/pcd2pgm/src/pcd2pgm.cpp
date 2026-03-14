#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/point_types.h>

#include "tf2/LinearMath/Quaternion.h"

class Pcd2pgm:public rclcpp::Node
{
private:
    std::string file_directory;
    std::string file_name;
    std::string pcd_file;
    std::string map_topic_name;
    const std::string pcd_format = ".pcd";
    nav_msgs::msg::OccupancyGrid map_topic_msg;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_topic_pub_;

    //最小和最大高度
    double thre_z_min ;
    double thre_z_max;
    int flag_pass_through;
    double map_resolution;
    double thre_radius;

    //半径滤波的点数阈值
    int thres_point_count;

    //直通滤波后数据指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_;

    //半径滤波后数据指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_;

    rclcpp::TimerBase::SharedPtr timer_;



public:
    Pcd2pgm(): rclcpp::Node("Pcd2pgm")
    {
        this->declare_parameter<double>("thre_z_min", 0.2);
        this->declare_parameter<double>("thre_z_max", 2.0);
        this->declare_parameter<int>("flag_pass_through", 0);
        this->declare_parameter<double>("thre_radius", 0.5);
        this->declare_parameter<double>("map_resolution", 0.05);
        this->declare_parameter<int>("thres_point_count", 10);
        this->declare_parameter<std::string>("file_directory", std::string("/home/nuc/navigationros2/ros2-humble/src/tools/pcd2pgm/save_pcd/"));
        this->declare_parameter<std::string>("file_name", std::string("GlobalMap"));
        this->declare_parameter<std::string>("map_topic_name", std::string("map"));
        
        this->get_parameter("thre_z_min",thre_z_min);
        this->get_parameter("thre_z_max",thre_z_max);
        this->get_parameter("flag_pass_through",flag_pass_through);
        this->get_parameter("thre_radius",thre_radius);
        this->get_parameter("map_resolution",map_resolution);
        this->get_parameter("thres_point_count",thres_point_count);
        this->get_parameter("map_topic_name",map_topic_name);
        this->get_parameter("file_directory",file_directory);
        this->get_parameter("file_name",file_name);

        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliable(); // 使用 Reliable 发布策略
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // 使用 Transient Local 持久性

        map_topic_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name, qos_profile);
        
        pcd_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_after_PassThrough_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_after_Radius_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();


        pcd_file = file_directory+file_name+pcd_format;

        RCLCPP_INFO(this->get_logger(),pcd_file.c_str());
          // 加载pcd文件
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud_) == -1) 
        {
            PCL_ERROR("Couldn't read file: %s \n", pcd_file.c_str());
            return;
        }
        std::cout << "初始点云数据点数：" << pcd_cloud_->points.size() << std::endl;

          //降采样
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pcd_cloud_);
        sor.setLeafSize(0.01, 0.01, 0.05);
        sor.filter(*pcd_cloud_);
        //对数据进行直通滤波
        PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through));
        //对数据进行半径滤波
        RadiusOutlierFilter(cloud_after_PassThrough_, thre_radius, thres_point_count);
        //转换为栅格地图数据并发布
        SetMapTopicMsg(cloud_after_Radius_, map_topic_msg);
        std::cout << "publishing map" << std::endl;
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Pcd2pgm::pubtimer_callback, this));
    
    }


    //直通滤波
    void PassThroughFilter(const double &thre_low, const double &thre_high,
                        const bool &flag_in);
    //半径滤波
    void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud_,
                            const double &radius, const int &thre_count);
    //转换为栅格地图数据并发布
    void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        nav_msgs::msg::OccupancyGrid &msg);

    void pubtimer_callback(void);
};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Pcd2pgm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void Pcd2pgm::PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in) {
  // 创建滤波器对象
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  //输入点云
  passthrough.setInputCloud(pcd_cloud_);
  //设置对z轴进行操作
  passthrough.setFilterFieldName("z");
  //设置滤波范围
  passthrough.setFilterLimits(thre_low, thre_high);
  // true表示保留滤波范围外，false表示保留范围内
  passthrough.setFilterLimitsNegative(flag_in);
  //执行滤波并存储
  passthrough.filter(*cloud_after_PassThrough_);
  // test 保存滤波后的点云到文件
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_filter.pcd",
                                      *cloud_after_PassThrough_);
  std::cout << "直通滤波后点云数据点数："
            << cloud_after_PassThrough_->points.size() << std::endl;
}

//半径滤波
void Pcd2pgm::RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0,
                         const double &radius, const int &thre_count) {
  //创建滤波器
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  //设置输入点云
  radiusoutlier.setInputCloud(pcd_cloud0);
  //设置半径,在该范围内找临近点
  radiusoutlier.setRadiusSearch(radius);
  //设置查询点的邻域点集数，小于该阈值的删除
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius_);
  // test 保存滤波后的点云到文件
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_radius_filter.pcd",
                                      *cloud_after_Radius_);
  std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius_->points.size()
            << std::endl;
}



//转换为栅格地图数据并发布
void Pcd2pgm::SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::msg::OccupancyGrid &msg) {
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = this->get_clock()->now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;
  //? ? ??
  double k_line =
      (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line =
      (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
      (thre_z_max - thre_z_min);

  if (cloud->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "pcd is empty!\n");
    return;
  }

  for (long unsigned int i = 0; i < cloud->points.size() - 1; i++) {
    if (i == 0) {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }
  // origin的确定
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min; 
  msg.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0,0,0);
  msg.info.origin.orientation.x = q.getX();
  msg.info.origin.orientation.y = q.getY();
  msg.info.origin.orientation.z = q.getZ();
  msg.info.origin.orientation.w = q.getW();
  //设置栅格地图大小
  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);
  //实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  RCLCPP_INFO(this->get_logger(), "data size = %d\n", msg.data.size());

  for (long unsigned int iter = 0; iter < cloud->points.size(); iter++) {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;
    // 栅格地图的占有概率[0,100]，这里设置为占据
    msg.data[i + j * msg.info.width] = 100;
    //    msg.data[i + j * msg.info.width] = int(255 * (cloud->points[iter].z *
    //    k_line + b_line)) % 255;
  }
}

void Pcd2pgm::pubtimer_callback(void)
{
    map_topic_pub_->publish(map_topic_msg);
}
