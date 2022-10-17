#ifndef ROSUNIT_FLOATSUB_HPP
#define ROSUNIT_FLOATSUB_HPP

#include "HEAR_ROS2/ROSUnit_Sub.hpp"
#include "std_msgs/msg/float32.hpp"

namespace HEAR{

class ROSUnitFloatSub : public ROSUnit_Sub<std_msgs::msg::Float32>{
private:
    void callback(const std_msgs::msg::Float32&);
public:
    ROSUnitFloatSub(rclcpp::Node *nh, const std::string& topic, int idx);
    TYPE getType(){ return TYPE::Float;}
};

ROSUnitFloatSub::ROSUnitFloatSub(rclcpp::Node *nh, const std::string& topic, int idx){
    sub_ = nh->create_subscription<std_msgs::msg::Float32>(topic, 10, std::bind(&ROSUnitFloatSub::callback, this, _1));
    _output_port = new OutputPort<float>(idx, 0);
    
    ((OutputPort<float>*)_output_port)->write(0);
    id_ = idx;
}
void ROSUnitFloatSub::callback(const std_msgs::msg::Float32& msg){
    ((OutputPort<float>*)_output_port)->write(msg.data);
}

}
#endif