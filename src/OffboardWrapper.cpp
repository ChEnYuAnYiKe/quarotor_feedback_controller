#include "OffboardWrapper.h"

OffboardWrapper::OffboardWrapper(geometry_msgs::PoseStamped position_setpoint,geometry_msgs::PoseStamped end_setpoint,
                                 std::string id, std::string node_id, std::string dataset) {
    uav_id = id;
    node_id = node_id;
    dataset_address = dataset;
    fly_time_delay = 0;
    fly_time_from_node_1 = 0;
    // Publisher
    m_Publisher.wrapper_local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            uav_id + "/mavros/setpoint_position/local", 10);
    m_Publisher.wrapper_vision_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(uav_id + "/mavros/vision_pose/pose",
                                                                                   10);
    m_Publisher.wrapper_attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(
            uav_id + "/mavros/setpoint_raw/attitude", 10);
    m_Publisher.wrapper_angluar_acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/angluar_acceleration",
                                                                                      10);

    m_Publisher.position_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(node_id + "/position_setpoint", 10);
    m_Publisher.velocity_setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>(node_id + "/velocity_setpoint", 10);
    m_Publisher.attitude_setpoint_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/attitude_setpoint", 10);
    m_Publisher.attitude_cureuler_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/attitude_euler", 10);
    m_Publisher.wrapper_status_pub = nh.advertise<std_msgs::Bool>(node_id + "/status", 10);
    m_Publisher.path_ref_pub = nh.advertise<nav_msgs::Path>(node_id + "/ref_path", 1);
    m_Publisher.path_fly_pub = nh.advertise<nav_msgs::Path>(node_id + "/fly_path", 1);

    m_Publisher.time_sync_pub = nh.advertise<std_msgs::Bool>(node_id + "/is_ok", 1);
    if (uav_id == "/uav1") {
        m_Publisher.fly_time_pub = nh.advertise<std_msgs::Time>("offb1_node/fly_time", 10);

    }
    m_Publisher.draw_ref_pub = nh.advertise<geometry_msgs::PoseStamped>(node_id + "/ref_point", 10);
    m_Publisher.draw_fly_pub = nh.advertise<geometry_msgs::PoseStamped>(node_id + "/fly_point", 10);

    // subscriber();
    subscriber();
    start_position_setpoint_ = position_setpoint;
    end_position_setpoint_ = end_setpoint;
    current_status_ = HOVER;

    hover_flag = 1;
    planning_flag = 1;
    end_flag = 1;
    time_sync_flag = 0;
}

OffboardWrapper::~OffboardWrapper() {
}

void OffboardWrapper::getEndPoint() {
//  ifstream fin(dataset_address);
//  string line, word;
//  vector<string> row;
//  vector<vector<string>> content;
//  int row_num = 0, col_num = 0;
//
//  while (getline(fin, line))
//  {
//    row.clear();
//    stringstream str(line);
//    col_num = 0;
//    while (getline(str, word, ','))
//    {
//      row.push_back(word);
//      content.push_back(row);
//      col_num += 1;
//    }
//    row_num += 1;
//  }
//  // std::cout << col_num << std::endl;
//  end_position_setpoint_.pose.position.x = stof(content[(row_num)*col_num-2][1]);
//  end_position_setpoint_.pose.position.y = stof(content[(row_num)*col_num-2][2]);
//  end_position_setpoint_.pose.position.z = stof(content[(row_num)*col_num-2][3]);
//  fin.close();
}

void OffboardWrapper::isAtSetpoint() {
    Eigen::Vector3d hp_(start_position_setpoint_.pose.position.x,
                        start_position_setpoint_.pose.position.y,
                        start_position_setpoint_.pose.position.z);
    Eigen::Vector3d dis_ = wrap_data.wrapper_current_position_ - hp_;

    if (dis_.norm() < 0.1) {
        if (hover_flag) {
            start_hover_t = ros::Time::now();
            hover_flag = 0;
            time_sync_flag = 1;
            node_1_time_pub_flg = 1;

        }
        if ((ros::Time::now() - start_hover_t).toSec() >= 5.0) {
            if (num_of_ok_drone == 1) {
                if (uav_id == "/uav1" && node_1_time_pub_flg == 1) {
                    fly_time = ros::Time::now() + ros::Duration(5);
                    std::cout << "uav 1 fly time:" << fly_time.toSec() << std::endl;
                    printf("uav1 get time");
                    node_1_time_pub_flg = 0;
                }
                if (fly_time_from_node_1 - ros::Time::now().toSec() < 0.1 &&
                    fly_time_from_node_1 - ros::Time::now().toSec() > 0) {
                    ros::Duration(fly_time_delay).sleep();
                    std::cout << uav_id << "get time:" << fly_time_from_node_1 << std::endl;
                    printf("enter planning!!\n");

                    std::cout << uav_id << "begin time:" << ros::Time::now().toSec() << std::endl;
                    // current_status_ = PLANNING;
                    current_status_ = HOVER;
                }

            }
            // enter planning
        }
    } else
        current_status_ = HOVER;
}

void OffboardWrapper::topicPublish() {
    wrap_data.thrust_attitude_cmd_.header.frame_id = "base_footprint";
    wrap_data.thrust_attitude_cmd_.header.stamp = ros::Time::now();
    m_Publisher.wrapper_attitude_pub_.publish(wrap_data.thrust_attitude_cmd_);

    m_Publisher.position_setpoint_pub.publish(wrap_data.pub_setpoint_position_);
    // std::cout << wrap_data.pub_setpoint_position_ << std::endl;
    m_Publisher.velocity_setpoint_pub.publish(wrap_data.pub_setpoint_velocity_);
    m_Publisher.attitude_setpoint_pub.publish(wrap_data.pub_setpoint_attitude_);
    m_Publisher.attitude_cureuler_pub.publish(wrap_data.pub_euler_attitude_);

    wrap_data.path_fly.poses.push_back(wrap_data.fly_path_point);
    wrap_data.path_ref.poses.push_back(wrap_data.ref_path_point);
    wrap_data.path_fly.header.frame_id = "odom";
    wrap_data.path_ref.header.frame_id = "odom";
    m_Publisher.path_ref_pub.publish(wrap_data.path_ref);
    m_Publisher.path_fly_pub.publish(wrap_data.path_fly);

    if (time_sync_flag == 1) {
        wrap_data.is_ok_.data = 1;
        m_Publisher.time_sync_pub.publish(wrap_data.is_ok_);
//      std::cout<<"TIME OK=========================================="<<std::endl;
        wrap_data.is_ok_.data = 0;
        time_sync_flag = 0;
    }
    if (uav_id == "/uav1") {
        wrap_data.fly_time_msg.data = fly_time;
        m_Publisher.fly_time_pub.publish(wrap_data.fly_time_msg);
    }
    wrap_data.is_ready_.data = current_status_;
    m_Publisher.wrapper_status_pub.publish(wrap_data.is_ready_);


}

void OffboardWrapper::subscriber() {
    m_Subscriber.wrapper_state_sub_ = nh.subscribe<mavros_msgs::State>(uav_id + "/mavros/state",
                                                                       10,
                                                                       &OffboardWrapper::stateCallback,
                                                                       this);
    // in real
     if(uav_id == ""){
        m_Subscriber.wrapper_vrpn_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/cyy/pose",
                                                       10,
                                                       &OffboardWrapper::visualCallback,
                                                       this);
    }
    else if(uav_id == "/uav2"){
        m_Subscriber.wrapper_vrpn_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/jiahao2/pose",
                                                       10,
                                                       &OffboardWrapper::visualCallback,
                                                       this);
    }
    // in simulator

    m_Subscriber.wrapper_velocity_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(uav_id + "/outer_velocity_vrpn",
                                                                                   10,
                                                                                   &OffboardWrapper::velocityCallback,
                                                                                   this);
    m_Subscriber.wrapper_acc_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(uav_id + "/outer_acc_vrpn",
                                                                              10,
                                                                              &OffboardWrapper::accCallback,
                                                                              this);
    m_Subscriber.wrapper_rc_state_sub_ = nh.subscribe<mavros_msgs::VFR_HUD>(uav_id + "/mavros/vfr_hud",
                                                                            10,
                                                                            &OffboardWrapper::rc_state_Callback,
                                                                            this);
    m_Subscriber.time_sync_sub1 = nh.subscribe<std_msgs::Bool>("offb1_node/is_ok", 10,
                                                               &OffboardWrapper::timesyncCallback1, this);
    m_Subscriber.time_sync_sub2 = nh.subscribe<std_msgs::Bool>("offb2_node/is_ok", 10,
                                                               &OffboardWrapper::timesyncCallback2, this);
    m_Subscriber.time_sync_sub3 = nh.subscribe<std_msgs::Bool>("offb3_node/is_ok", 10,
                                                               &OffboardWrapper::timesyncCallback3, this);
    m_Subscriber.time_sync_sub4 = nh.subscribe<std_msgs::Bool>("offb4_node/is_ok", 10,
                                                               &OffboardWrapper::timesyncCallback4, this);
    m_Subscriber.time_sync_sub5 = nh.subscribe<std_msgs::Bool>("offb5_node/is_ok", 10,
                                                               &OffboardWrapper::timesyncCallback5, this);

    m_Subscriber.fly_time_sub = nh.subscribe<std_msgs::Time>("offb1_node/fly_time", 10,
                                                             &OffboardWrapper::flytimeCallback, this);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_id + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_id + "/mavros/set_mode");
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    if (uav_id == "/uav0")
        m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb_node/status",
                                                                       10,
                                                                       &OffboardWrapper::statusCallback,
                                                                       this);
    else if (uav_id == "/uav1")
        m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb1_node/status",
                                                                       10,
                                                                       &OffboardWrapper::statusCallback,
                                                                       this);
    else if (uav_id == "/uav2")
        m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb2_node/status",
                                                                       10,
                                                                       &OffboardWrapper::statusCallback,
                                                                       this);
    else if (uav_id == "/uav3")
        m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb3_node/status",
                                                                       10,
                                                                       &OffboardWrapper::statusCallback,
                                                                       this);
    else if (uav_id == "/uav4")
        m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb4_node/status",
                                                                       10,
                                                                       &OffboardWrapper::statusCallback,
                                                                       this);
    else if (uav_id == "/uav5")
        m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb5_node/status",
                                                                       10,
                                                                       &OffboardWrapper::statusCallback,
                                                                       this);
}

void OffboardWrapper::rc_state_Callback(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
    wrap_data.rc_state = msg->heading;
    // std::cout<<msg->heading<<std::endl;
}

void OffboardWrapper::flytimeCallback(const std_msgs::Time::ConstPtr &msg) {
    std_msgs::Time time = *msg;
    fly_time_from_node_1 = time.data.toSec();
}

void OffboardWrapper::timesyncCallback1(const std_msgs::Bool::ConstPtr &msg) {
    std_msgs::Bool_<allocator<void>> is_ok = *msg;
    num_of_ok_drone++;
    std::cout << num_of_ok_drone << std::endl;
}

void OffboardWrapper::timesyncCallback2(const std_msgs::Bool::ConstPtr &msg) {
    std_msgs::Bool_<allocator<void>> is_ok = *msg;
    num_of_ok_drone++;
    std::cout << num_of_ok_drone << std::endl;
}

void OffboardWrapper::timesyncCallback3(const std_msgs::Bool::ConstPtr &msg) {
    std_msgs::Bool_<allocator<void>> is_ok = *msg;
    num_of_ok_drone++;
    std::cout << num_of_ok_drone << std::endl;
}

void OffboardWrapper::timesyncCallback4(const std_msgs::Bool::ConstPtr &msg) {
    std_msgs::Bool_<allocator<void>> is_ok = *msg;
    num_of_ok_drone++;
    std::cout << num_of_ok_drone << std::endl;
}

void OffboardWrapper::timesyncCallback5(const std_msgs::Bool::ConstPtr &msg) {
    std_msgs::Bool_<allocator<void>> is_ok = *msg;
    num_of_ok_drone++;
    std::cout << num_of_ok_drone << std::endl;
}

void OffboardWrapper::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    wrapper_current_state_ = *msg;
    wrap_data.current_state_ = wrapper_current_state_.mode;
}

void OffboardWrapper::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    geometry_msgs::TwistStamped wrapper_current_velo;
    wrapper_current_velo = *msg;
    // x y switch from betaflight!!!!!!!!!!!
    Eigen::Vector3d cur_velocity(wrapper_current_velo.twist.linear.x,
                                 wrapper_current_velo.twist.linear.y,
                                 wrapper_current_velo.twist.linear.z);
    wrap_data.wrapper_current_velocity_ = cur_velocity;
}

void OffboardWrapper::visualCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    geometry_msgs::PoseStamped wrapper_current_vrpn_ = *msg;
    tf::Quaternion rq;
    Vector3d cur_position_(wrapper_current_vrpn_.pose.position.x,
                           wrapper_current_vrpn_.pose.position.y,
                           wrapper_current_vrpn_.pose.position.z);
    // Vector3d cur_velocity_;
    // cur_velosity_ = (cur_position_ - wrap_data.wrapper_last_position_) / (ros::Time::now() - wrap_data.last_v_time).toNSec();
    // wrap_data.last_v_time = ros::Time::now();
    // wrap_data.wrapper_last_position_ = cur_position_;
    // wrap_data.wrapper_current_velocity_ = cur_velosity_;
    // std::cout << (ros::Time::now() - wrap_data.last_v_time).toNSec() << std::endl;

    wrap_data.wrapper_current_position_ = cur_position_;

    tf::quaternionMsgToTF(wrapper_current_vrpn_.pose.orientation, rq);
    wrap_data.wrapper_current_orientation(0) = wrapper_current_vrpn_.pose.orientation.x;
    wrap_data.wrapper_current_orientation(1) = wrapper_current_vrpn_.pose.orientation.y;
    wrap_data.wrapper_current_orientation(2) = wrapper_current_vrpn_.pose.orientation.z;
    wrap_data.wrapper_current_orientation(3) = wrapper_current_vrpn_.pose.orientation.w;
    tf::Matrix3x3(rq).getRPY(wrap_data.wrapper_current_attitude_[0],
                             wrap_data.wrapper_current_attitude_[1],
                             wrap_data.wrapper_current_attitude_[2]);

    // geometry_msgs::PoseStamped wrapper_vision_pos_ = wrapper_current_vrpn_;
    // wrapper_vision_pos_.header.stamp = ros::Time::now();
    // m_Publisher.wrapper_vision_pos_pub_.publish(wrapper_vision_pos_);
}

void OffboardWrapper::accCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    geometry_msgs::TwistStamped wrapper_current_acc;
    wrapper_current_acc = *msg;
    // x y switch from betaflight!!!!!!!!!!!
    Eigen::Vector3d cur_acc(wrapper_current_acc.twist.linear.x,
                            wrapper_current_acc.twist.linear.y,
                            wrapper_current_acc.twist.linear.z);
    wrap_data.wrapper_current_acc_ = cur_acc;
}

// in real
//  void OffboardWrapper::visualCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
//    geometry_msgs::PoseStamped wrapper_current_vrpn_ = *msg;
//    Vector3d cur_position_(wrapper_current_vrpn_.pose.position.x,
//                           wrapper_current_vrpn_.pose.position.y,
//                           wrapper_current_vrpn_.pose.position.z);

//   wrap_data.wrapper_current_position_ = cur_position_;

//   geometry_msgs::PoseStamped wrapper_vision_pos_ = wrapper_current_vrpn_;
//   wrapper_vision_pos_.header.stamp = ros::Time::now();
//   m_Publisher.wrapper_vision_pos_pub_.publish(wrapper_vision_pos_);
// }
// in simulator
//void OffboardWrapper::visualCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//    nav_msgs::Odometry wrapper_current_vrpn_ = *msg;
//    Vector3d cur_position_;
//    if (uav_id == "/uav0") {
//        cur_position_ = Vector3d(wrapper_current_vrpn_.pose.pose.position.x + 0,
//                                 wrapper_current_vrpn_.pose.pose.position.y + 0,
//                                 wrapper_current_vrpn_.pose.pose.position.z);
//    } else if (uav_id == "/uav1") {
//        cur_position_ = Vector3d(wrapper_current_vrpn_.pose.pose.position.x + 0,
//                                 wrapper_current_vrpn_.pose.pose.position.y + 2,
//                                 wrapper_current_vrpn_.pose.pose.position.z);
//    } else if (uav_id == "/uav2") {
//        cur_position_ = Vector3d(wrapper_current_vrpn_.pose.pose.position.x,
//                                 wrapper_current_vrpn_.pose.pose.position.y + 3,
//                                 wrapper_current_vrpn_.pose.pose.position.z);
//    } else if (uav_id == "/uav3") {
//        cur_position_ = Vector3d(wrapper_current_vrpn_.pose.pose.position.x,
//                                 wrapper_current_vrpn_.pose.pose.position.y + 4,
//                                 wrapper_current_vrpn_.pose.pose.position.z);
//    } else if (uav_id == "/uav4") {
//        cur_position_ = Vector3d(wrapper_current_vrpn_.pose.pose.position.x + 0,
//                                 wrapper_current_vrpn_.pose.pose.position.y + 5,
//                                 wrapper_current_vrpn_.pose.pose.position.z);
//    } else if (uav_id == "/uav5") {
//        cur_position_ = Vector3d(wrapper_current_vrpn_.pose.pose.position.x + 0,
//                                 wrapper_current_vrpn_.pose.pose.position.y + 6,
//                                 wrapper_current_vrpn_.pose.pose.position.z);
//    }
//
//
//    wrap_data.wrapper_current_position_ = cur_position_;
//}

void OffboardWrapper::statusCallback(const std_msgs::Bool::ConstPtr &msg) {
    ready_flag = (*msg).data;
}

void OffboardWrapper::run() {
    wrap_data.fly_path_point = start_position_setpoint_;
    wrap_data.ref_path_point = start_position_setpoint_;
    QuadrotorFeedbackController c1(start_position_setpoint_, &wrap_data);
    QuadrotorMPCController c2(&wrap_data);
//  getEndPoint(); // end_position_setpoint_ from this function
    c2.readCsvData(dataset_address);
    QuadrotorFeedbackController c3(end_position_setpoint_, &wrap_data);

    ros::Rate rate(LOOP_FREQUENCY);

    while (ros::ok()) {
        // std::cout << "end_position_setpoint_.pose.position.x: " << end_position_setpoint_.pose.position.x << std::endl;
        // std::cout << "end_position_setpoint_.pose.position.y: " << end_position_setpoint_.pose.position.y << std::endl;
        // std::cout << "end_position_setpoint_.pose.position.z: " << end_position_setpoint_.pose.position.z << std::endl;
        // start_planning_t_ = ros::Time::now();

        switch (current_status_) {
            case HOVER:
                // printf("enter hover!!\n");
                isAtSetpoint();
                c1.loadLatestData();
                if (!wrap_data.rc_state) {
                    c1.reset_error_sum_both_pv();
                    // ROS_INFO("RESET I");
                }
                c1.positionControlFeedback();
                c1.velocityControlFeedback();
                start_planning_t_ = ros::Time::now();
                break;

            case READY:
                printf("enter ready!!\n");
                if (ready_flag) current_status_ = PLANNING;
                c1.loadLatestData();
                c1.positionControlFeedback();
                c1.velocityControlFeedback();
                start_planning_t_ = ros::Time::now();
//            m_Publisher.wrapper_local_pos_pub_.publish(wrap_data.pub_setpoint_position_);
                break;


            case PLANNING:
                if (planning_flag) {
                    for (int i = 0; i < 5; i++) {
                        c2.thrust_ave_ += wrap_data.thrust_eval[i] / 5;
                    }
                    c2.replan_delta_time = ros::Time::now();
                    planning_flag = 0;
                    c2.ff_cmd.is_done = false;
                }
                ros::spinOnce();
                c2.current_time_ = (ros::Time::now() - start_planning_t_).toSec();
                c2.loadFeedforwardData();
                c2.loadLatestData();
                c2.MPCControl();
                if (c2.ff_cmd.is_done)
                    current_status_ = END;
//      topicPublish();
                m_Publisher.draw_fly_pub.publish(wrap_data.fly_path_point);
                m_Publisher.draw_ref_pub.publish(wrap_data.ref_path_point);
                break;

            case END:
                printf("aggressive flight is done!!\n");
                c3.loadLatestData();
                c3.positionControlFeedback();
                c3.velocityControlFeedback();
//            m_Publisher.wrapper_local_pos_pub_.publish(wrap_data.pub_setpoint_position_);
                break;
        }

        topicPublish();
        ros::spinOnce();
        rate.sleep();
    }
}