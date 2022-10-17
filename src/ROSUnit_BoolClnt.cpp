#include "HEAR_ROS/ROSUnit_BoolClnt.hpp"

namespace HEAR{

ROSUnitBoolClient::ROSUnitBoolClient(ros::NodeHandle &nh, std::string t_name){
    m_client = nh.serviceClient<std_srvs::SetBool>(t_name);
}

bool ROSUnitBoolClient::process(bool data){
    std_srvs::SetBool msg;
    msg.request.data = data;
    return m_client.call(msg);
}

}