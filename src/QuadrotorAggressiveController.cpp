#include "QuadrotorAggressiveController.h"

QuadrotorAggressiveController::QuadrotorAggressiveController(struct DataCentre *wrap_data_ptr) {
    data_ptr = wrap_data_ptr;

    thrust_ave_ = 0;
    data_ptr_ = 0;

    position_error_sum_ = Eigen::Vector3d(0, 0, 0);
    velocity_error_sum_ = Eigen::Vector3d(0, 0, 0);

    // planning PID params init
    kp_planning_x_ = 11.0;
    kp_planning_y_ = 10.0;
    kp_planning_z_ = 9.5;
    kp_planning_vx_ = 8.0;
    kp_planning_vy_ = 7.0;
    kp_planning_vz_ = 1.3;

    ki_planning_x_ = 0;
    ki_planning_y_ = 0;
    ki_planning_z_ = 0.01;
    ki_planning_vx_ = 0;
    ki_planning_vy_ = 0;
    ki_planning_vz_ = 0;

    kd_planning_x_ = 0;
    kd_planning_y_ = 0;
    kd_planning_z_ = 0;
    kd_planning_vx_ = 0;
    kd_planning_vy_ = 0;
    kd_planning_vz_ = 0;

    kp_attitude_x = 9.0;
    kp_attitude_y = 8.5;
    kp_attitude_z = 5.5;

    kp_acc_t = 0.010;
}

QuadrotorAggressiveController::~QuadrotorAggressiveController() {
}

void QuadrotorAggressiveController::readCsvData(std::string dataset) {
    ifstream fin(dataset);
    string line, word;
    vector<string> row;
    vector<vector<string>> content;
    int row_num = 0, col_num = 0;

    while (getline(fin, line)) {
        row.clear();
        stringstream str(line);
        col_num = 0;
        while (getline(str, word, ',')) {
            row.push_back(word);
            content.push_back(row);
            col_num += 1;
        }
        row_num += 1;
    }
    vector<string> rad[18];
    int pp = 0;
    for (int i = col_num * 3 - 1; i < content.size(); i += col_num) {
        rad[0].push_back(std::to_string(pp * 0.03));
        pp += 1;
        rad[1].push_back(content[i][1]);
        rad[2].push_back(content[i][2]);
        rad[3].push_back(content[i][3]);
        rad[4].push_back(content[i][8]);
        rad[5].push_back(content[i][9]);
        rad[6].push_back(content[i][10]);
        rad[7].push_back(content[i][5]);
        rad[8].push_back(content[i][6]);
        rad[9].push_back(content[i][7]);
        rad[10].push_back(content[i][4]);
        rad[11].push_back(content[i][20]);
        rad[12].push_back(content[i][21]);
        rad[13].push_back(content[i][22]);
        rad[14].push_back(content[i][23]);
        rad[15].push_back(content[i][11]);
        rad[16].push_back(content[i][12]);
        rad[17].push_back(content[i][13]);
        // rad[18].push_back(content[i][14]);
        // rad[19].push_back(content[i][15]);
        // rad[20].push_back(content[i][16]);
    }
    csv_size_ = rad[0].size();
    for (int k = 0; k < rad[0].size(); k++) {
        for (int j = 0; j < 18; j++) {
            csv_data_[k][j] = stof(rad[j][k]);
            cout << csv_data_[k][j] << " ";
        }
        cout << endl;
    }
    fin.close();
}

void QuadrotorAggressiveController::loadFeedforwardData() {

    while ((data_ptr_ < csv_size_ - 1) && (csv_data_[data_ptr_ + 1][0] < current_time_)) {
        data_ptr_ += 1;
    }

    Eigen::Vector3d data_position(csv_data_[data_ptr_][1], csv_data_[data_ptr_][2], csv_data_[data_ptr_][3]);
    Eigen::Vector3d data_velocity(csv_data_[data_ptr_][4], csv_data_[data_ptr_][5], csv_data_[data_ptr_][6]);
    Eigen::Vector3d data_attitude(csv_data_[data_ptr_][7], csv_data_[data_ptr_][8], csv_data_[data_ptr_][9]);
    Eigen::Vector3d data_rate(csv_data_[data_ptr_][15], csv_data_[data_ptr_][16], csv_data_[data_ptr_][17]);
    Eigen::Vector3d data_acc(csv_data_[data_ptr_][7], csv_data_[data_ptr_][8], csv_data_[data_ptr_][9]);
    ff_cmd.position_cmd = data_position;
    ff_cmd.velocity_cmd = data_velocity;
    ff_cmd.attitude_cmd = data_attitude;
    ff_cmd.rate_cmd = data_rate;
    ff_cmd.acc_cmd = data_acc;
    ff_cmd.thrust =
            csv_data_[data_ptr_][11] + csv_data_[data_ptr_][12] + csv_data_[data_ptr_][13] + csv_data_[data_ptr_][14];
    // tf::Quaternion rq;
    // geometry_msgs::PoseStamped data_quaternion;
    // data_quaternion.pose.orientation.x = csv_data_[data_ptr_][7];
    // data_quaternion.pose.orientation.y = csv_data_[data_ptr_][8];
    // data_quaternion.pose.orientation.z = csv_data_[data_ptr_][9];
    // data_quaternion.pose.orientation.w = csv_data_[data_ptr_][10];
    // tf::quaternionMsgToTF(data_quaternion.pose.orientation, rq);
    // tf::Matrix3x3(rq).getRPY(ff_cmd.attitude_cmd[0],
    //                          ff_cmd.attitude_cmd[1],
    //                          ff_cmd.attitude_cmd[2]);
    if (data_ptr_ == csv_size_ - 1)
        ff_cmd.is_done = 1;
    else
        ff_cmd.is_done = 0;
}

void QuadrotorAggressiveController::loadLatestData() {
    current_position_ = data_ptr->wrapper_current_position_;
    current_velocity_ = data_ptr->wrapper_current_velocity_;
    current_attitude_ = data_ptr->wrapper_current_attitude_;
    current_acc_ = data_ptr->wrapper_current_acc_;
    // current_state_ = data_ptr->current_state_;
}

void QuadrotorAggressiveController::aggressiveControl() {
    Eigen::Vector3d position_cmd_ = ff_cmd.position_cmd;
    Eigen::Vector3d position_error_ = position_cmd_ - current_position_;
    Eigen::Vector3d velocity_cmd_ = ff_cmd.velocity_cmd;
    Eigen::Vector3d velocity_error_ = velocity_cmd_ - current_velocity_;
    double phi_ = current_attitude_[0];
    double theta_ = current_attitude_[1];
    double psi_ = current_attitude_[2];
    Eigen::Matrix3d R_E_B_;
    R_E_B_ << cos(psi_), sin(psi_), ZERO,
            -sin(psi_), cos(psi_), ZERO,
            ZERO, ZERO, ONE;
    position_error_ = R_E_B_ * position_error_;
    velocity_error_ = R_E_B_ * velocity_error_;
    position_error_sum_ += position_error_ / LOOP_FREQUENCY;
    velocity_error_sum_ += velocity_error_ / LOOP_FREQUENCY;
    Eigen::Matrix3d k_position, k_velocity, k_attitude;
    k_position << kp_planning_x_, ZERO, ZERO,
            ZERO, kp_planning_y_, ZERO,
            ZERO, ZERO, kp_planning_z_;
    k_velocity << kp_planning_vx_, ZERO, ZERO,
            ZERO, kp_planning_vy_, ZERO,
            ZERO, ZERO, kp_planning_vz_;
    k_attitude << kp_attitude_x, ZERO, ZERO,
            ZERO, kp_attitude_y, ZERO,
            ZERO, ZERO, kp_attitude_z;
    Eigen::Vector3d acc_fb = k_position * position_error_ + k_velocity * velocity_error_;

    double phi_cmd_ = ff_cmd.attitude_cmd[0], theta_cmd_ = ff_cmd.attitude_cmd[1], psi_cmd_ = ff_cmd.attitude_cmd[2];
    Eigen::Matrix3d R_;
    R_ << cos(theta_cmd_) * cos(psi_cmd_), cos(theta_cmd_) * sin(psi_cmd_), -sin(theta_cmd_),
            sin(phi_cmd_) * sin(theta_cmd_) * cos(psi_cmd_) - cos(phi_cmd_) * sin(psi_cmd_),
            sin(phi_cmd_) * sin(theta_cmd_) * sin(psi_cmd_) + cos(phi_cmd_) * cos(psi_cmd_),
            sin(phi_cmd_) * cos(theta_cmd_),
            cos(phi_cmd_) * sin(theta_cmd_) * cos(psi_cmd_) + sin(phi_cmd_) * sin(psi_cmd_),
            cos(phi_cmd_) * sin(theta_cmd_) * sin(psi_cmd_) - sin(phi_cmd_) * cos(psi_cmd_),
            cos(phi_cmd_) * cos(theta_cmd_);

    double thrust_cmd_ = ff_cmd.thrust;
    if (thrust_cmd_ < 0.01)
        thrust_cmd_ = 9.81;
    Eigen::Vector3d z_cmd_(0, 0, thrust_cmd_);

    Eigen::Vector3d acc_ff = R_.transpose() * z_cmd_;
//     Eigen::Vector3d ff_cmd.acc_cmd;
//    // cout << acc_ff << endl;
//    // cout << ff_cmd.acc_cmd << endl;
//    double psi_cmd_ = 0.0;
//    Eigen::Vector3d acc_ff = ff_cmd.acc_cmd;
//    acc_ff[2] += 9.81;
//    cout << acc_ff << endl;
    Eigen::Vector3d acc_des = acc_ff + acc_fb;

    double zi_term = ki_planning_z_ * position_error_[2] + ki_planning_vz_ * velocity_error_[2];
    if (zi_term > 5)
        zi_term = 5;
    if (zi_term < -5)
        zi_term = -5;
    acc_des[2] += zi_term;

    if (acc_des[2] < 0.1)
        acc_des[2] = 0.1;

    Eigen::Vector3d zb_des = acc_des / acc_des.norm();
    Eigen::Vector3d yc(-sin(psi_cmd_), cos(psi_cmd_), 0);
    Eigen::Vector3d xb_des = yc.cross(zb_des) / (yc.cross(zb_des)).norm();
    Eigen::Vector3d yb_des = zb_des.cross(xb_des);

    double psi_des = atan2(xb_des[1], xb_des[0]);
    double theta_des = asin(-xb_des[2]);
    double phi_des = atan(yb_des[2] / zb_des[2]);
    double thrust_des = acc_des.norm() * thrust_ave_ / 9.81;

    double thrust_bias = kp_acc_t * (acc_des.norm() - current_acc_.norm());
    if (thrust_bias > 0.1)
        thrust_bias = 0.1;
    if (thrust_bias < -0.1)
        thrust_bias = -0.1;
    thrust_des += thrust_bias;
    cout << "thrust_bias:" << thrust_bias << endl;

    if (psi_des > 0.1)
        psi_des = 0.1;
    if (psi_des < -0.1)
        psi_des = -0.1;
    if (phi_des > 1.3)
        phi_des = 1.3;
    if (phi_des < -1.3)
        phi_des = -1.3;
    if (theta_des > 1.3)
        theta_des = 1.3;
    if (theta_des < -1.3)
        theta_des = -1.3;
    if (thrust_des > 0.95)
        thrust_des = 0.95;
    if (thrust_des < 0)
        thrust_des = 0;

    Eigen::Vector3d attitude_cmd(phi_des, theta_des, psi_des);
    Eigen::Vector3d rate_fb = k_attitude * (attitude_cmd - current_attitude_);
    Eigen::Vector3d rate_des = ff_cmd.rate_cmd + rate_fb;

    thrust_attitude_cmd_.thrust = thrust_des;
    // thrust_attitude_cmd_.thrust = thrust_ave_;
    thrust_attitude_cmd_.body_rate.x = rate_des[0];
    thrust_attitude_cmd_.body_rate.y = rate_des[1];
    // dirty fix, in order to reduce the yaw rate gap caused by type_mask shift,send yaw attitude error all time.
    thrust_attitude_cmd_.body_rate.z = rate_des[2];
    //   thrust_attitude_cmd_.body_rate.x = 0;
    // thrust_attitude_cmd_.body_rate.y = 0.1;
    // thrust_attitude_cmd_.body_rate.z = 0;
    thrust_attitude_cmd_.type_mask = 128;

    data_ptr->thrust_attitude_cmd_ = thrust_attitude_cmd_;

    data_ptr->pub_setpoint_position_.pose.position.x = position_cmd_[0];
    data_ptr->pub_setpoint_position_.pose.position.y = position_cmd_[1];
    data_ptr->pub_setpoint_position_.pose.position.z = position_cmd_[2];
    data_ptr->pub_setpoint_position_.header.stamp = ros::Time::now();
    data_ptr->pub_setpoint_position_.header.frame_id = "odom";

    data_ptr->pub_setpoint_velocity_.twist.linear.x = velocity_cmd_[0];
    data_ptr->pub_setpoint_velocity_.twist.linear.y = velocity_cmd_[1];
    data_ptr->pub_setpoint_velocity_.twist.linear.z = velocity_cmd_[2];
    data_ptr->pub_setpoint_velocity_.header.stamp = ros::Time::now();
    data_ptr->pub_setpoint_velocity_.header.frame_id = "odom";

    data_ptr->pub_euler_attitude_.vector.x = current_attitude_[0] * 180 / PI;
    data_ptr->pub_euler_attitude_.vector.y = current_attitude_[1] * 180 / PI;
    data_ptr->pub_euler_attitude_.vector.z = current_attitude_[2] * 180 / PI;
    data_ptr->pub_euler_attitude_.header.stamp = ros::Time::now();

    // data_ptr->pub_euler_attitude_.vector.x = acc_des[0];
    // data_ptr->pub_euler_attitude_.vector.y = acc_des[1];
    // data_ptr->pub_euler_attitude_.vector.z = acc_des[2];
    // data_ptr->pub_euler_attitude_.header.stamp = ros::Time::now();

    data_ptr->pub_setpoint_attitude_.vector.x = phi_des * 180 / PI;
    data_ptr->pub_setpoint_attitude_.vector.y = theta_des * 180 / PI;
    data_ptr->pub_setpoint_attitude_.vector.z = psi_des * 180 / PI;
    data_ptr->pub_setpoint_attitude_.header.stamp = ros::Time::now();

    data_ptr->pub_new_velocity.header.stamp = ros::Time::now();
    data_ptr->pub_new_velocity.vector.x = current_velocity_[0];
    data_ptr->pub_new_velocity.vector.y = current_velocity_[1];
    data_ptr->pub_new_velocity.vector.z = current_velocity_[2];

    data_ptr->ref_path_point.pose.position.x = position_cmd_[0];
    data_ptr->ref_path_point.pose.position.y = position_cmd_[1];
    data_ptr->ref_path_point.pose.position.z = position_cmd_[2];
    data_ptr->ref_path_point.header.stamp = ros::Time::now();
    data_ptr->ref_path_point.header.frame_id = "odom";

    data_ptr->fly_path_point.pose.position.x = current_position_[0];
    data_ptr->fly_path_point.pose.position.y = current_position_[1];
    data_ptr->fly_path_point.pose.position.z = current_position_[2];
    data_ptr->fly_path_point.header.stamp = ros::Time::now();
    data_ptr->fly_path_point.header.frame_id = "odom";
}
