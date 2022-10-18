#ifndef ROSUNIT_QUATSUB_HPP
#define ROSUNIT_QUATSUB_HPP

#include "HEAR_ROS2/Subscribers/ROSUnit_Sub.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace HEAR{
class ROSUnitQuatSub : public ROSUnit_Sub{
private:
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_;
    void callback(const geometry_msgs::msg::Quaternion& msg);
public:
    ROSUnitQuatSub (rclcpp::Node::SharedPtr nh, const std::string& topic, int idx);
    TYPE getType(){return TYPE::Float3;}
};

ROSUnitQuatSub::ROSUnitQuatSub (rclcpp::Node::SharedPtr nh, const std::string& topic, int idx){
    sub_ = nh->create_subscription<geometry_msgs::msg::Quaternion>(topic, 10, std::bind(&ROSUnitQuatSub::callback, this, _1));
    _output_port = new OutputPort<tf2::Quaternion>(idx, 0);
    id_ = idx;
}

void ROSUnitQuatSub::callback(const geometry_msgs::msg::Quaternion& msg){
    tf2::Quaternion data(msg.x, msg.y, msg.z, msg.w);
    ((OutputPort<tf2::Quaternion>*)_output_port)->write(data);
}

}

#endif