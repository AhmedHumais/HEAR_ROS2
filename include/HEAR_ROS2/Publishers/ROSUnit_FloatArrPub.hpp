#ifndef ROSUNIT_FLOATARRPUB_HPP
#define ROSUNIT_FLOATARRPUB_HPP

#include "HEAR_ROS2/Publishers/ROSUnit_Pub.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <vector>

namespace HEAR{

class ROSUnitFloatArrPub : public ROSUnit_Pub{
private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
public:
    ROSUnitFloatArrPub(rclcpp::Node::SharedPtr nh, const std::string& topic_name, int idx){
        pub_ = nh->create_publisher<std_msgs::msg::Float32MultiArray>(topic_name, 10);
        _input_port = new InputPort<std::vector<float>>(idx, 0);
        id_ = idx;
    }

    TYPE getType(){ return TYPE::FloatVec;}
    void process(){
        if (_input_port != NULL){
            std_msgs::msg::Float32MultiArray msg;
            ((InputPort<std::vector<float>>*)_input_port)->read(msg.data);
            pub_->publish(msg);
        }
    }
};

}

#endif