
#ifndef ROSUNITINTPUB_HPP
#define ROSUNITINTPUB_HPP


#include "HEAR_ROS/ROSUnit_Pub.hpp"
#include "std_msgs/Int32.h"

namespace HEAR{

class ROSUnitIntPub : public ROSUnit_Pub{
public:
    ROSUnitIntPub(ros::NodeHandle& nh, const std::string& topic_name, int idx) {
        pub_ = nh.advertise<std_msgs::Int32>(topic_name, 10);
        _input_port = new InputPort<int>(idx, 0);
        id_ = idx; 
    }
    TYPE getType(){return TYPE::Float;}
    void process(){
        if (_input_port != NULL){
            std_msgs::Int32 msg;
            int x=0;
            ((InputPort<int>*)_input_port)->read(x);
            msg.data = x;
            pub_.publish(msg);
        }
    }
};

}

#endif