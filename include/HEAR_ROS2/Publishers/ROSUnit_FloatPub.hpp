
#ifndef ROSUNITFLOATPUB_HPP
#define ROSUNITFLOATPUB_HPP


#include "HEAR_ROS2/ROSUnit_Pub.hpp"
#include "std_msgs/msg/float32.hpp"

namespace HEAR{

class ROSUnitFloatPub : public ROSUnit_Pub<std_msgs::msg::Float32>{
public:
    ROSUnitFloatPub(rclcpp::Node *nh, const std::string& topic_name, int idx) {
        pub_ = nh->create_publisher<std_msgs::msg::Float32>(topic_name, 10);
        _input_port = new InputPort<float>(idx, 0);
        id_ = idx; 
    }
    TYPE getType(){return TYPE::Float;}
    void process(){
        if (_input_port != NULL){
            std_msgs::msg::Float32 msg;
            ((InputPort<float>*)_input_port)->read(msg.data);
            pub_->publish(msg);
        }
    }
};

}

#endif