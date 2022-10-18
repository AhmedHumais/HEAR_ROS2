
#ifndef ROSUNIT_RESETSRV_HPP
#define ROSUNIT_RESETSRV_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace HEAR{
class ROSUnit_ResetServer {
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_server;
    ResetTrigger* ext_trig;
    void srv_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);
public:
    ROSUnit_ResetServer(rclcpp::Node::SharedPtr);
    ResetTrigger* registerServer(const std::string&);
 
};

}

#endif