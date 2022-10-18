#ifndef ROSUNIT_EMPTYCLNT
#define ROSUNIT_EMPTYCLNT

#include <chrono>
#include <cinttypes>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace HEAR{

class ROSUnitEmptyClient {
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_client;
public:
    ROSUnitEmptyClient(rclcpp::Node::SharedPtr, const std::string&);
    bool process();
};

}

#endif