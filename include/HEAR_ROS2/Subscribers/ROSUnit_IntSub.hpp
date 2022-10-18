#ifndef ROSUNIT_INTSUB_HPP
#define ROSUNIT_INTSUB_HPP

#include "HEAR_ROS2/Subscribers/ROSUnit_Sub.hpp"
#include "std_msgs/msg/int32.hpp"

namespace HEAR{

class ROSUnitIntSub : public ROSUnit_Sub{
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    void callback(const std_msgs::msg::Int32::SharedPtr);
public:
    ROSUnitIntSub(rclcpp::Node::SharedPtr nh, const std::string& topic, int idx);
    TYPE getType(){ return TYPE::Int;}
};

ROSUnitIntSub::ROSUnitIntSub(rclcpp::Node::SharedPtr nh, const std::string& topic, int idx){
    sub_ = nh->create_subscription<std_msgs::msg::Int32>(topic, 10, std::bind(&ROSUnitIntSub::callback, this, _1));
    _output_port = new OutputPort<int>(idx, 0);
    
    ((OutputPort<int>*)_output_port)->write(0);
    id_ = idx;
}
void ROSUnitIntSub::callback(const std_msgs::msg::Int32::SharedPtr msg){
    ((OutputPort<int>*)_output_port)->write(msg->data);
}

}
#endif