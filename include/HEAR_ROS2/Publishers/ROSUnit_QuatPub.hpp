#ifndef ROSUNIT_QUATPUB_HPP
#define ROSUNIT_QUATPUB_HPP

#include "HEAR_ROS2/Publishers/ROSUnit_Pub.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "HEAR_core/Vector3D.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace HEAR{

class ROSUnitQuatPub : public ROSUnit_Pub{
private:
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_;
public:
    ROSUnitQuatPub(rclcpp::Node::SharedPtr nh, const std::string& topic_name, int idx){
        pub_ = nh->create_publisher<geometry_msgs::msg::Quaternion>(topic_name, 10);
        _input_port = new InputPort<tf2::Quaternion>(idx, 0);
        id_ = idx;
    }

    TYPE getType(){ return TYPE::Float3;}
    
    void process(){
        if (_input_port != NULL){
            geometry_msgs::msg::Quaternion msg;
            tf2::Quaternion data;
            ((InputPort<tf2::Quaternion>*)_input_port)->read(data);
            msg.x = data.x(); msg.y = data.y(), msg.z = data.z(), msg.w = data.w();
            pub_->publish(msg);
        }
    }
};

}

#endif