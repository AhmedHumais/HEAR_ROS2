#include "HEAR_ROS2/Custom/ROSUnit_PoseProvider.hpp"

namespace HEAR{

ROSUnit_PoseProvider::ROSUnit_PoseProvider(rclcpp::Node::SharedPtr nh): nh_(nh){
}


std::vector<ExternalOutputPort<Vector3D<float>>*> ROSUnit_PoseProvider::registerOptiPose(std::string t_name){
    // m_server = nh_.advertiseService("/set_height_offset", &ROSUnit_PoseProvider::srv_callback, this);
    rot_offset.setRPY(0.0, 0.0, M_PI/2.0);
    trans_offset.setZero();
    
    opti_pos_port = new ExternalOutputPort<Vector3D<float>>(0);
    opti_pos_port->write(Vector3D<float>(0,0,0));
    opti_vel_port = new ExternalOutputPort<Vector3D<float>>(0);
    opti_vel_port->write(Vector3D<float>(0,0,0));
    opti_ori_port = new ExternalOutputPort<Vector3D<float>>(0);
    opti_ori_port->write(Vector3D<float>(0,0,0));
    opti_sub = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(t_name, 10, std::bind(&ROSUnit_PoseProvider::callback_opti_pose, this, _1));
    return std::vector<ExternalOutputPort<Vector3D<float>>*>{opti_pos_port, opti_vel_port, opti_ori_port};
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuOri(std::string t_name){
    imu_ori_port = new ExternalOutputPort<Vector3D<float>>(0);
    imu_ori_port->write(Vector3D<float>(0,0,0));
    xsens_ori_sub = nh_->create_subscription<geometry_msgs::msg::QuaternionStamped>(t_name, 10, std::bind(&ROSUnit_PoseProvider::callback_ori, this, _1));
    return imu_ori_port;
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuAngularRate(std::string t_name){
    imu_angular_rt_port = new ExternalOutputPort<Vector3D<float>>(0);
    imu_angular_rt_port->write(Vector3D<float>(0,0,0));
    xsens_ang_vel_sub = nh_->create_subscription<geometry_msgs::msg::Vector3Stamped>(t_name, 10, std::bind(&ROSUnit_PoseProvider::callback_angular_vel, this, _1));
    return imu_angular_rt_port;
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuAcceleration(std::string t_name){
    imu_acc_port = new ExternalOutputPort<Vector3D<float>>(0);
    imu_acc_port->write(Vector3D<float>(0,0,0));
    xsens_free_acc_sub = nh_->create_subscription<geometry_msgs::msg::Vector3Stamped>(t_name, 10, std::bind(&ROSUnit_PoseProvider::callback_free_acc, this, _1));
    return imu_acc_port;
}

// bool ROSUnit_PoseProvider::srv_callback(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res) {
//     trans_offset.setZ(req.data);
//     return true;
// }

void ROSUnit_PoseProvider::callback_opti_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    
    tf2::Vector3 vel;
    auto pos = tf2::Vector3({msg->pose.position.x, msg->pose.position.y, msg->pose.position.z});
    auto calib_pos = rot_offset*pos - trans_offset;

    Vector3D<float> vec = {(float)calib_pos.x(), (float)calib_pos.y(), (float)calib_pos.z()};

    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w ));

    R_mat = rot_offset * R_mat * rot_offset.transpose();
    tf2Scalar yaw, pitch, roll;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec_ori = {(float)roll, (float)pitch, (float)yaw};

    // velocity calculation
    if(first_read == 0){
        first_read = 1;
        prevT = msg->header.stamp;
        prev_pos = pos;
        vel = tf2::Vector3(0, 0, 0);
        prev_diff = vel;
    }else{
        auto _dt = ((rclcpp::Time)msg->header.stamp - prevT).seconds();
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
        prevT = msg->header.stamp;
    }
    opti_vel = rot_offset*vel;
    ////////////////////////

    opti_pos_port->write(vec);
    opti_vel_port->write(Vector3D<float>(opti_vel.x(), opti_vel.y(), opti_vel.z()));
    opti_ori_port->write(vec_ori);
}

void ROSUnit_PoseProvider::callback_ori(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg){
    
    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));

    tf2Scalar yaw, roll, pitch;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec = {(float)roll, (float)pitch, (float)yaw};
    imu_ori_port->write(vec);
}

void ROSUnit_PoseProvider::callback_angular_vel(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg){
    Vector3D<float> vec = {(float)msg->vector.x, (float)msg->vector.y, (float)msg->vector.z};

    imu_angular_rt_port->write(vec);
}

void ROSUnit_PoseProvider::callback_free_acc(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg){
    Vector3D<float> vec = {(float)msg->vector.x, (float)msg->vector.y, (float)msg->vector.z};

    imu_acc_port->write(vec);
}

}