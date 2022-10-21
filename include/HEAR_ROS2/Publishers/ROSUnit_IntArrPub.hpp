#ifndef __ROSUNIT_INTARRPUB_H__
#define __ROSUNIT_INTARRPUB_H__

#include "HEAR_ROS2/Publishers/ROSUnit_Pub.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <vector>

namespace HEAR{

class ROSUnitIntArrPub : public ROSUnit_Pub{
private:
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
public:
    ROSUnitIntArrPub(rclcpp::Node::SharedPtr nh, const std::string& topic_name, int idx){
        pub_ = nh->create_publisher<std_msgs::msg::Int32MultiArray>(topic_name, 10);
        _input_port = new InputPort<std::vector<int>>(idx, 0);
        id_ = idx;
    }

    TYPE getType(){ return TYPE::FloatVec;}
    void process(){
        if (_input_port != NULL){
            std_msgs::msg::Int32MultiArray msg;
            std::vector<int> dat;
            ((InputPort<std::vector<int>>*)_input_port)->read(dat);
            msg.data = std::vector<int32_t>(dat.begin(), dat.end());
            pub_->publish(msg);
        }
    }
};

}
#endif // __ROSUNIT_INTARRPUB_H__