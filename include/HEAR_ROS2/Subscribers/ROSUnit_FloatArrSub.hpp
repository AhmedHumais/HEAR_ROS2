#ifndef __ROSUNIT_FLOATARRSUB_H__
#define __ROSUNIT_FLOATARRSUB_H__

#include "HEAR_ROS2/Subscribers/ROSUnit_Sub.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace HEAR{

class ROSUnitFloatArrSub : public ROSUnit_Sub{
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr);
public:
    ROSUnitFloatArrSub(rclcpp::Node::SharedPtr nh, std::string topic, int idx);
    TYPE getType(){ return TYPE::FloatVec;}
};

ROSUnitFloatArrSub::ROSUnitFloatArrSub(rclcpp::Node::SharedPtr nh, std::string topic, int idx){
    sub_ = nh->create_subscription<std_msgs::msg::Float32MultiArray>(topic, 10, std::bind(&ROSUnitFloatArrSub::callback, this, _1));
    _output_port = new OutputPort<std::vector<float>>(idx, 0);
    
    id_ = idx;
}
void ROSUnitFloatArrSub::callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    ((OutputPort<std::vector<float>>*)_output_port)->write(msg->data);
}

}
#endif // __ROSUNIT_FLOATARRSUB_H__