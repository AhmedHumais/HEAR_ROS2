#ifndef ROSUNIT_POINTSUB_HPP
#define ROSUNIT_POINTSUB_HPP

#include "HEAR_ROS2/Subscribers/ROSUnit_Sub.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace HEAR{
class ROSUnitPointSub : public ROSUnit_Sub{
private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
    void callback(const geometry_msgs::msg::Point::SharedPtr msg);
public:
    ROSUnitPointSub (rclcpp::Node::SharedPtr nh, const std::string& topic, int idx);
    TYPE getType(){return TYPE::Float3;}
};

ROSUnitPointSub::ROSUnitPointSub (rclcpp::Node::SharedPtr nh, const std::string& topic, int idx){
    sub_ = nh->create_subscription<geometry_msgs::msg::Point>(topic, 10, std::bind(&ROSUnitPointSub::callback, this, _1));
    _output_port = new OutputPort<Vector3D<float>>(idx, 0);
    id_ = idx;
}

void ROSUnitPointSub::callback(const geometry_msgs::msg::Point::SharedPtr msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ((OutputPort<Vector3D<float>>*)_output_port)->write(data);
}

}

#endif