#include "HEAR_ROS2/Services/ROSUnit_ResetSrv.hpp"

namespace HEAR{

ROSUnit_ResetServer::ROSUnit_ResetServer(rclcpp::Node::SharedPtr nh) : nh_(nh) {
    ext_trig = new ResetTrigger;

}

ResetTrigger* ROSUnit_ResetServer::registerServer(const std::string &service_topic){
    m_server = nh_->create_service<std_srvs::srv::Empty>(service_topic, std::bind(&ROSUnit_ResetServer::srv_callback, this, _2));  

    return ext_trig;
}

void ROSUnit_ResetServer::srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response){
    ext_trig->resetCallback();
}

}