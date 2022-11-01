#ifndef ROSUNIT_SLAM_HPP
#define ROSUNIT_SLAM_HPP

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Transform.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace HEAR{

class ROSUnit_SLAM {
private:
    const float PEAK_THRESH = 0.35;
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_offset_srv;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string ref_frame = "map";
    rclcpp::Time prevT;
    uint8_t first_read = 0;
    InputPort<Vector3D<float>>* pos_inp_port;
    InputPort<Vector3D<float>>* ori_inp_port;

    ExternalOutputPort<Vector3D<float>>* pos_out_port;
    ExternalOutputPort<Vector3D<float>>* vel_out_port;
    ExternalOutputPort<Vector3D<float>>* ori_out_port;
    tf2::Transform offset_tf;
    tf2::Vector3 slam_pos, prev_pos, slam_vel, prev_diff, _hold;
    tf2::Matrix3x3 slam_rot;    
    bool to_map = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

public:
    ROSUnit_SLAM(rclcpp::Node::SharedPtr nh, bool use_map = false);
    ~ROSUnit_SLAM(){}
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerSLAM(const std::string& t_name);
    void connectInputs(OutputPort<Vector3D<float>>* pos_port, OutputPort<Vector3D<float>>* ori_port);
    
};

}

#endif