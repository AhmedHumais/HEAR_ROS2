#include "HEAR_ROS2/Clients/ROSUnit_EmptyClnt.hpp"

namespace HEAR{

ROSUnitEmptyClient::ROSUnitEmptyClient(rclcpp::Node::SharedPtr nh, const std::string & t_name) : nh_(nh){
    m_client = nh_->create_client<std_srvs::srv::Empty>(t_name);
}

bool ROSUnitEmptyClient::process(){
    if(!m_client->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_ERROR(nh_->get_logger(), "Service not found");
        return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    
    auto res_future = m_client->async_send_request(request); 
    for(uint8_t i=0; i<100; i++){
        if (rclcpp::spin_until_future_complete(nh_,res_future, std::chrono::milliseconds(10)) == 
                            rclcpp::FutureReturnCode::SUCCESS ){
            return true;
        }
    }
    return false;
}

}