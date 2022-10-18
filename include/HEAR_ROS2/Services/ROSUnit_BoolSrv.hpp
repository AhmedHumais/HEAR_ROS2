#ifndef ROSUNIT_BOOLSRV_HPP
#define ROSUNIT_BOOLSRV_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

using std::placeholders::_2;

namespace HEAR{
class ROSUnit_BoolServer {
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_server;
    UpdateTrigger* ext_trig;
    void srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
public:
    ROSUnit_BoolServer(rclcpp::Node::SharedPtr);
    UpdateTrigger* registerServer(const std::string&);
    
};

}

#endif