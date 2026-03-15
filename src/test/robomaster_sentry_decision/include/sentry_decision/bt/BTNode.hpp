#ifndef BTNODE_HPP
#define BTNODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>

// ===== 关键修复 1 =====
// 必须包含完整定义，不能只前向声明
#include "sentry_decision/Blackboard.hpp"

class RegionManager;

enum class BTStatus {
    SUCCESS,
    FAILURE,
    RUNNING
};

class BTNode {
public:
    virtual ~BTNode() = default;
    
    virtual BTStatus execute(std::shared_ptr<Blackboard> blackboard, 
                             std::shared_ptr<RegionManager> region_manager) = 0;
    
    virtual void reset() {}
    
    void setNodeName(const std::string& name) { node_name_ = name; }
    std::string getNodeName() const { return node_name_; }
    
protected:
    // ================================
    // 工具函数：判断是否到达目标点
    // ================================
    static bool isAtTarget(std::shared_ptr<Blackboard> blackboard, 
                           const geometry_msgs::msg::Point& target, 
                           double tolerance = 50.0) 
    {
        if (!blackboard) return false;

        // ===== 关键修复 2 =====
        // 现在 Blackboard 已完整定义，可安全访问成员
        double dx = blackboard->x - target.x;
        double dy = blackboard->y - target.y;

        return std::sqrt(dx * dx + dy * dy) <= tolerance;
    }
    
private:
    std::string node_name_;
};

// ================================
// 复合节点基类
// ================================
class BTComposite : public BTNode {
public:
    void addChild(std::shared_ptr<BTNode> child) {
        children_.push_back(child);
    }
    
    void clearChildren() {
        children_.clear();
    }
    
    size_t getChildCount() const {
        return children_.size();
    }
    
    std::shared_ptr<BTNode> getChild(size_t index) {
        if (index < children_.size()) {
            return children_[index];
        }
        return nullptr;
    }
    
protected:
    std::vector<std::shared_ptr<BTNode>> children_;
};

#endif // BTNODE_HPP

