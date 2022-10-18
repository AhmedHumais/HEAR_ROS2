#ifndef ROSUNIT_POSEPROVIDER_HPP
#define ROSUNIT_POSEPROVIDER_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/ExternalTrigger.hpp"

using std::placeholders::_1;

namespace HEAR{

class ROSUnit_PoseProvider{
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr opti_sub; 
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr xsens_ori_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr xsens_ang_vel_sub, xsens_free_acc_sub;
    // ros::ServiceServer m_server;
    
    ExternalOutputPort<Vector3D<float>>* opti_pos_port;
    ExternalOutputPort<Vector3D<float>>* opti_vel_port;
    ExternalOutputPort<Vector3D<float>>* opti_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_acc_port;
    ExternalOutputPort<Vector3D<float>>* imu_angular_rt_port;
    void callback_opti_pose(const geometry_msgs::msg::PoseStamped::SharedPtr );
    void callback_ori(const geometry_msgs::msg::QuaternionStamped::SharedPtr );
    void callback_free_acc(const geometry_msgs::msg::Vector3Stamped::SharedPtr );
    void callback_angular_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr );
    // bool srv_callback(hear_msgs::set_float::Request&, hear_msgs::set_float::Response&);
    tf2::Matrix3x3 rot_offset;
    tf2::Vector3 trans_offset;

    tf2::Vector3 opti_pos, prev_pos, opti_vel, prev_diff, _hold;
    rclcpp::Time prevT;
    uint8_t first_read = 0;
    const float PEAK_THRESH = 0.35;

public:
    void process(){}
    ROSUnit_PoseProvider(rclcpp::Node::SharedPtr nh);
    ~ROSUnit_PoseProvider(){}
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerOptiPose(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuOri(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAngularRate(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAcceleration(std::string t_name);
};

}

#endif