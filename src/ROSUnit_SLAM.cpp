#include "HEAR_ROS2/Custom/ROSUnit_SLAM.hpp"

namespace HEAR{

ROSUnit_SLAM::ROSUnit_SLAM(rclcpp::Node::SharedPtr nh, bool use_map) : nh_(nh), to_map(use_map){
    pos_inp_port = new InputPort<Vector3D<float>>(0, 0);
    ori_inp_port = new InputPort<Vector3D<float>>(0, 0);

    tf2::Matrix3x3 rot;
    rot.setIdentity();
    offset_tf = tf2::Transform(rot);
    slam_pos = tf2::Vector3(0,0,0); slam_rot.setIdentity();
}

std::vector<ExternalOutputPort<Vector3D<float>>*> ROSUnit_SLAM::registerSLAM(const std::string& t_name){
    
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    set_offset_srv = nh_->create_service<std_srvs::srv::SetBool>("/set_map_frame_offset", std::bind(&ROSUnit_SLAM::srv_callback, this, _1, _2));
    
    odom_sub = nh_->create_subscription<nav_msgs::msg::Odometry>(t_name, 10, std::bind(&ROSUnit_SLAM::odom_callback, this, std::placeholders::_1));
    
    pos_out_port = new ExternalOutputPort<Vector3D<float>>(0);
    vel_out_port = new ExternalOutputPort<Vector3D<float>>(0);
    ori_out_port = new ExternalOutputPort<Vector3D<float>>(0);

    return std::vector<ExternalOutputPort<Vector3D<float>>*>{pos_out_port, ori_out_port, vel_out_port};
}

void ROSUnit_SLAM::connectInputs(OutputPort<Vector3D<float>>* pos_port, OutputPort<Vector3D<float>>* ori_port){
    pos_inp_port->connect(pos_port);
    ori_inp_port->connect(ori_port);
}

void ROSUnit_SLAM::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
    tf2::Vector3 pos, vel;
    tf2::Matrix3x3 rot;

    geometry_msgs::msg::PoseStamped slam_pose, tf_pose;
    slam_pose.header = odom_msg->header;
    slam_pose.pose = odom_msg->pose.pose;

    if(to_map){
        try{
            tf_pose = tf_buffer_->transform(slam_pose, ref_frame, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(nh_->get_logger(), "Failure %s\n", ex.what()); //Print exception which was caught
        }
        pos = {tf_pose.pose.position.x, tf_pose.pose.position.y, tf_pose.pose.position.z};
        auto ori = tf2::Quaternion(tf_pose.pose.orientation.x, tf_pose.pose.orientation.y, tf_pose.pose.orientation.z, tf_pose.pose.orientation.w);
        rot.setRotation(ori);
    }
    else{
        pos = tf2::Vector3(slam_pose.pose.position.x, slam_pose.pose.position.y, slam_pose.pose.position.z);
        rot.setRotation(tf2::Quaternion(slam_pose.pose.orientation.x, slam_pose.pose.orientation.y, slam_pose.pose.orientation.z, slam_pose.pose.orientation.w));
    }
    slam_pos = offset_tf*pos;
    slam_rot = offset_tf.getBasis()*rot*(offset_tf.getBasis().transpose());

    // velocity calculation
    if(first_read == 0){
        first_read = 1;
        prevT = slam_pose.header.stamp;
        prev_pos = pos;
        vel = tf2::Vector3(0, 0, 0);
        prev_diff = vel;
    }else{
        auto _dt = ( (rclcpp::Time)slam_pose.header.stamp - prevT).seconds();
        auto diff = (pos - prev_pos)/_dt;
        vel = diff;
        if(first_read == 1){
            first_read = 2;
            prev_diff = diff;
        }
        auto d_diff = diff - prev_diff;
        if(abs(d_diff.x()) > PEAK_THRESH || abs(d_diff.y()) > PEAK_THRESH || abs(d_diff.z()) > PEAK_THRESH){
            vel = _hold;
        }
        else{
            _hold = diff;
        }
        prev_diff = diff;
        prev_pos = pos;
        prevT = slam_pose.header.stamp;
    }
    slam_vel = offset_tf.getBasis()*vel;
    ////////////////////////

    double r, p, y;
    slam_rot.getRPY(r, p, y);
    
    pos_out_port->write(Vector3D<float>(slam_pos.x(), slam_pos.y(), slam_pos.z()));
    vel_out_port->write(Vector3D<float>(slam_vel.x(), slam_vel.y(), slam_vel.z()));
    ori_out_port->write(Vector3D<float>(r, p, y));
}

void ROSUnit_SLAM::srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    
    if(request->data){
    Vector3D<float> angs, trans;
    pos_inp_port->read(trans);
    ori_inp_port->read(angs);
    
    tf2::Matrix3x3 rot;
    rot.setEulerYPR(angs.z, angs.y, angs.x);
    offset_tf.setBasis(rot*(slam_rot.transpose()));
    offset_tf.setOrigin(tf2::Vector3(trans.x, trans.y, trans.z) - offset_tf.getBasis()*slam_pos);
    }

    response->success =true;
}

}