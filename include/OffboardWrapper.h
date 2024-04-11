#pragma once

#include "QuadrotorFeedbackController.h"
#include "QuadrotorMPCController.h"
#include "QuadrotorAggressiveController.h"
#include <vector>
#include <fstream>
#include <iostream>

class OffboardWrapper {
private:
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped start_position_setpoint_, end_position_setpoint_;
    ros::Time start_planning_t_, start_hover_t, fly_time;

    // std::string current_status_;
    std::string uav_id, node_id;
    std::string dataset_address;
    int current_status_;

    int num_of_ok_drone = 0;
    //flag
    bool hover_flag;
    bool planning_flag;
    bool end_flag;
    bool ready_flag;
    bool time_sync_flag;
    double fly_time_delay;

    void isAtSetpoint();

    void topicPublish();

public:
    OffboardWrapper(geometry_msgs::PoseStamped position_setpoint,geometry_msgs::PoseStamped end_setpoint,
                    std::string id, std::string node_id, std::string dataset);

    ~OffboardWrapper();

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;
    double fly_time_from_node_1;
    double node_1_time_pub_flg = 0;

    mavros_msgs::State wrapper_current_state_;

    struct SubsciberWrapper {
        ros::Subscriber wrapper_state_sub_;
        ros::Subscriber wrapper_vrpn_sub_;
        ros::Subscriber wrapper_fused_sub_;
        ros::Subscriber wrapper_velocity_sub_;
        ros::Subscriber wrapper_status_sub;
        ros::Subscriber time_sync_sub1, time_sync_sub2, time_sync_sub3, time_sync_sub4, time_sync_sub5;
        ros::Subscriber fly_time_sub;
        ros::Subscriber wrapper_rc_state_sub_;
        ros::Subscriber wrapper_acc_sub_;
    } m_Subscriber;

    struct PublisherWrapper {
        ros::Publisher wrapper_local_pos_pub_;
        ros::Publisher wrapper_attitude_pub_;
        ros::Publisher wrapper_vision_pos_pub_;
        ros::Publisher wrapper_status_pub;
        ros::Publisher wrapper_angluar_acc_pub;

        ros::Publisher position_setpoint_pub;
        ros::Publisher velocity_setpoint_pub;
        ros::Publisher attitude_setpoint_pub;
        ros::Publisher attitude_cureuler_pub;
        ros::Publisher path_ref_pub;
        ros::Publisher path_fly_pub;
        ros::Publisher time_sync_pub;
        ros::Publisher fly_time_pub;
        ros::Publisher draw_fly_pub;
        ros::Publisher draw_ref_pub;
    } m_Publisher;

    struct DataCentre wrap_data;

    void subscriber();

    void run();

    void flytimeCallback(const std_msgs::Time::ConstPtr &msg);

    void timesyncCallback1(const std_msgs::Bool::ConstPtr &msg);

    void timesyncCallback2(const std_msgs::Bool::ConstPtr &msg);

    void timesyncCallback3(const std_msgs::Bool::ConstPtr &msg);

    void timesyncCallback4(const std_msgs::Bool::ConstPtr &msg);

    void timesyncCallback5(const std_msgs::Bool::ConstPtr &msg);

    void stateCallback(const mavros_msgs::State::ConstPtr &msg);

    void rc_state_Callback(const mavros_msgs::VFR_HUD::ConstPtr &msg);

    void visualCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
//    void visualCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void fused_pathCallback(const nav_msgs::Path::ConstPtr &msg);

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void accCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);


    void statusCallback(const std_msgs::Bool::ConstPtr &msg);

    void getEndPoint();
};
