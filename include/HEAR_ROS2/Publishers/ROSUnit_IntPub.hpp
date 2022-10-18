
#ifndef ROSUNITINTPUB_HPP
#define ROSUNITINTPUB_HPP


#include "HEAR_ROS2/Publishers/ROSUnit_Pub.hpp"
#include "std_msgs/msg/int32.hpp"

namespace HEAR{

class ROSUnitIntPub : public ROSUnit_Pub{
private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
public:
    ROSUnitIntPub(rclcpp::Node::SharedPtr nh, const std::string& topic_name, int idx) {
        pub_ = nh->create_publisher<std_msgs::msg::Int32>(topic_name, 10);
        _input_port = new InputPort<int>(idx, 0);
        id_ = idx; 
    }
    TYPE getType(){return TYPE::Float;}
    void process(){
        if (_input_port != NULL){
            std_msgs::msg::Int32 msg;
            int x=0;
            ((InputPort<int>*)_input_port)->read(x);
            msg.data = x;
            pub_->publish(msg);
        }
    }
};

}

#endif