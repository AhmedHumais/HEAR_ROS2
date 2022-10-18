#ifndef ROSUNIT_BOOLCLNT
#define ROSUNIT_BOOLCLNT

#include <chrono>
#include <cinttypes>
#include <memory>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace HEAR{

class ROSUnitBoolClient {
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_client;
public:
    ROSUnitBoolClient(rclcpp::Node::SharedPtr, const std::string&);
    bool process(bool m_data);
};

}

#endif