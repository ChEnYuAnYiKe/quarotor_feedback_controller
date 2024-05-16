#pragma once

#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <Eigen/Dense>
#include "ParamLoad.h"

#include <sensor_msgs/Imu.h>

#define LOOP_FREQUENCY 200
#define PI 3.1415
#define ZERO 0
#define ONE 1

#define OFFBOARD "OFFBOARD"

enum state {
    HOVER, READY, PLANNING, END
};

struct DataCentre {
    bool rc_state;
    ros::Time begin_time;
    Eigen::Vector3d wrapper_last_position_;
    ros::Time last_v_time;
    Eigen::Vector3d wrapper_current_position_, wrapper_current_velocity_, wrapper_current_attitude_, wrapper_current_acc_;
    Eigen::Vector4d wrapper_current_orientation;
    std::string current_state_;

    mavros_msgs::AttitudeTarget thrust_attitude_cmd_;
    std_msgs::Bool is_ready_;
    geometry_msgs::PoseStamped pub_setpoint_position_;
    geometry_msgs::TwistStamped pub_setpoint_velocity_;
    geometry_msgs::Vector3Stamped pub_setpoint_attitude_, pub_euler_attitude_, pub_new_velocity;
    geometry_msgs::Vector3Stamped attitude_cmd_kp_, attitude_cmd_ki_, attitude_cmd_kd_;
    geometry_msgs::PoseStamped ref_path_point, fly_path_point;
    nav_msgs::Path path_ref, path_fly;
    std_msgs::Bool is_ok_;
    std_msgs::Time fly_time_msg;

    double thrust_eval[5];
};

class QuadrotorFeedbackController {
private:
    // const ros::NodeHandle &nh_;
    ros::NodeHandle nh2;
    //data
    geometry_msgs::PoseStamped position_setpoint_;
    geometry_msgs::TwistStamped velocity_setpoint_;
    mavros_msgs::AttitudeTarget thrust_attitude_cmd_;


    std::string current_state_;
    std::string current_status_;

    // hover PID params
    double kp_hover_x_, kp_hover_y_, kp_hover_z_, kp_hover_vx_, kp_hover_vy_, kp_hover_vz_;
    double ki_hover_x_, ki_hover_y_, ki_hover_z_, ki_hover_vx_, ki_hover_vy_, ki_hover_vz_;
    double kd_hover_x_, kd_hover_y_, kd_hover_z_, kd_hover_vx_, kd_hover_vy_, kd_hover_vz_;

    //thrust eval
    int eval_ptr_;

    Eigen::Vector3d current_position_, current_velocity_, current_attitude_;
    Eigen::Vector3d position_error_sum_, velocity_error_sum_;
    Eigen::Vector3d position_last_error_, position_diff_;
    Eigen::Vector3d velocity_last_error_, velocity_diff_;

public:
    struct DataCentre *data_ptr;

    //function
    QuadrotorFeedbackController(geometry_msgs::PoseStamped position_setpoint, struct DataCentre *wrap_data_ptr);

    ~QuadrotorFeedbackController();

    void loadLatestData();

    void reset_error_sum_both_pv();

    void positionControlFeedback();

    void velocityControlFeedback();

    void get_params(const ros::NodeHandle &nh);
};


