#include "HEAR_ROS2/Services/ROSUnit_BoolSrv.hpp"

namespace HEAR{

ROSUnit_BoolServer::ROSUnit_BoolServer(rclcpp::Node::SharedPtr nh) : nh_(nh){
    ext_trig = new UpdateTrigger;
}

UpdateTrigger* ROSUnit_BoolServer::registerServer(const std::string &service_topic){
    m_server = nh_->create_service<std_srvs::srv::SetBool>(service_topic, std::bind(&ROSUnit_BoolServer::srv_callback, this, _1, _2));  
    return ext_trig;
}

void ROSUnit_BoolServer::srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    BoolMsg msg;
    msg.data = request->data;
    ext_trig->UpdateCallback((UpdateMsg*)&msg);
    response->success = true;
}

}