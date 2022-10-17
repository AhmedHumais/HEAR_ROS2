#ifndef ROSUNIT_INTSUB_HPP
#define ROSUNIT_INTSUB_HPP

#include "HEAR_ROS/ROSUnit_Sub.hpp"
#include "std_msgs/Int32.h"

namespace HEAR{

class ROSUnitIntSub : public ROSUnit_Sub{
private:
    void callback(const std_msgs::Int32::ConstPtr&);
public:
    ROSUnitIntSub(ros::NodeHandle& nh, const std::string& topic, int idx);
    TYPE getType(){ return TYPE::Int;}
};

ROSUnitIntSub::ROSUnitIntSub(ros::NodeHandle& nh, const std::string& topic, int idx){
    sub_ = nh.subscribe<std_msgs::Int32>(topic, 10, &ROSUnitIntSub::callback, this);
    _output_port = new OutputPort<int>(idx, 0);
    
    ((OutputPort<int>*)_output_port)->write(0);
    id_ = idx;
}
void ROSUnitIntSub::callback(const std_msgs::Int32::ConstPtr& msg){
    ((OutputPort<int>*)_output_port)->write(msg->data);
}

}
#endif