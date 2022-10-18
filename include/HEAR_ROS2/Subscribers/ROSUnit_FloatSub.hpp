#ifndef ROSUNIT_FLOATSUB_HPP
#define ROSUNIT_FLOATSUB_HPP

#include "HEAR_ROS2/Subscribers/ROSUnit_Sub.hpp"
#include "std_msgs/msg/float32.hpp"

namespace HEAR{

class ROSUnitFloatSub : public ROSUnit_Sub{
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    void callback(const std_msgs::msg::Float32&) const;
public:
    ROSUnitFloatSub(rclcpp::Node::SharedPtr nh, std::string topic, int idx);
    TYPE getType(){ return TYPE::Float;}
};

ROSUnitFloatSub::ROSUnitFloatSub(rclcpp::Node::SharedPtr nh, std::string topic, int idx){
    sub_ = nh->create_subscription<std_msgs::msg::Float32>(topic, 10, std::bind(&ROSUnitFloatSub::callback, this, _1));
    _output_port = new OutputPort<float>(idx, 0);
    
    ((OutputPort<float>*)_output_port)->write(0);
    id_ = idx;
}
void ROSUnitFloatSub::callback(const std_msgs::msg::Float32& msg) const{
    ((OutputPort<float>*)_output_port)->write(msg.data);
}

}
#endif