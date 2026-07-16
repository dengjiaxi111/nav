#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/json_export.h"
#include <signal.h>
#include <string.h>

#include "mytree/UpdateInfo.hpp"
#include "mytree/UpdateEnemy.hpp"
#include "mytree/ModeControl.hpp"
#include "mytree/TriggerRelocalization.hpp"
#include "mytree/WaitingForStart.hpp"
#include "mytree/Patrol.hpp"
#include "mytree/GoSupply.hpp"
#include "mytree/ChaseEnemy.hpp"
#include "mytree/AttackDPoint.hpp"
#include "mytree/Defensive.hpp"
#include "mytree/GetBuff.hpp"

using namespace std;

namespace myBT
{
    class MytreeNode:public rclcpp::Node
    {
    public:
        MytreeNode():Node("mytree")
        {
            RCLCPP_INFO(this->get_logger(),"Node mytree RUNNING!");
        }
    private:

    };

}


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = make_shared<myBT::MytreeNode>();

    std::string tree_path = ament_index_cpp::get_package_share_directory("mytree") + "/tree/maintree.xml";

    BT::BehaviorTreeFactory factory;

    // 读取参数
    int tick_rate = 20;
    node->declare_parameter<int>("tick_rate", 20);
    node->get_parameter("tick_rate", tick_rate);

    // 注册自定义 BT 节点
    factory.registerBuilder<myBT::UpdateInfo>("UpdateInfo",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::UpdateInfo>(name, config, node);
    });
    
    factory.registerBuilder<myBT::UpdateEnemy>("UpdateEnemy",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::UpdateEnemy>(name, config, node);
    });

    factory.registerBuilder<myBT::ModeControl>("ModeControl",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::ModeControl>(name, config, node);
    });

    factory.registerBuilder<myBT::TriggerRelocalization>("TriggerRelocalization",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::TriggerRelocalization>(name, config, node);
    });
    

    factory.registerBuilder<myBT::GetBuff>("GetBuff",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::GetBuff>(name, config, node);
    });

    factory.registerBuilder<myBT::WaitingForStart>("WaitingForStart",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::WaitingForStart>(name, config, node);
    });

    factory.registerBuilder<myBT::GoSupply>("GoSupply",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::GoSupply>(name, config, node);
    });

    factory.registerBuilder<myBT::AttackDPoint>("AttackDPoint",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::AttackDPoint>(name, config, node);
    });

    factory.registerBuilder<myBT::Patrol>("Patrol",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::Patrol>(name, config, node);
    });

    factory.registerBuilder<myBT::Defensive>("Defensive",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::Defensive>(name, config, node);
    });

    factory.registerBuilder<myBT::ChaseEnemy>("ChaseEnemy",
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<myBT::ChaseEnemy>(name, config, node);
    });
    auto tree = factory.createTreeFromFile(tree_path); 


    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        tree.tickOnce();
        tree.sleep(chrono::milliseconds(1000 / tick_rate));
    }
    rclcpp::shutdown();
    return 0;
}
