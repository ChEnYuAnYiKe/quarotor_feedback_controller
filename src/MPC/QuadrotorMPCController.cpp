//
// Created by zfg on 23-5-5.
//

#include <fstream>
#include "QuadrotorMPCController.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

void mpc_acado_params::getState(const int node_index, Eigen::Ref<Eigen::Matrix<float, N_state, 1>> return_state) {
    return_state = acado_states_.col(node_index).cast<float>();
}

void mpc_acado_params::getStates(Eigen::Ref<Eigen::Matrix<float, N_state, N_sample + 1>> return_states) {
    return_states = acado_states_.cast<float>();
}

void mpc_acado_params::getInput(const int node_index, Eigen::Ref<Eigen::Matrix<float, N_input, 1>> return_input) {
    return_input = acado_inputs_.col(node_index).cast<float>();
}

void mpc_acado_params::getInputs(Eigen::Ref<Eigen::Matrix<float, N_input, N_sample>> return_input) {
    return_input = acado_inputs_.cast<float>();
}

bool mpc_acado_params::prepare() {
    acado_preparationStep();
    acado_is_prepared_ = true;

    return true;
}

mpc_acado_params::mpc_acado_params() {
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    acado_initializeSolver();
    const Eigen::Matrix<float, N_state, 1> hover_state = (Eigen::Matrix<float, N_state, 1>() << 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0).finished();
    acado_initial_state_ = hover_state;
    acado_states_ = hover_state.replicate(1, N_sample + 1);
    acado_inputs_ = kHoverInput_.replicate(1, N_sample);
    acado_reference_states_.block(0, 0, N_state, N_sample) = hover_state.replicate(1, N_sample);
    acado_reference_states_.block(N_state, 0, N_state_cost - N_state, N_sample) = Eigen::Matrix<float,
            N_state_cost - N_state, N_sample>::Zero();
    acado_reference_states_.block(N_state_cost, 0, N_input, N_sample) =
            kHoverInput_.replicate(1, N_sample);
    acado_reference_end_state_.segment(0, N_state) = hover_state;

    acado_reference_end_state_.segment(N_state, N_state_cost - N_state) =
            Eigen::Matrix<float, N_state_cost - N_state, 1>::Zero();
    acado_W_ = W_.replicate(1, N_sample);
    acado_W_end_ = WN_;

    acado_initializeNodesByForwardSimulation();
    acado_preparationStep();
    acado_is_prepared_ = true;

}

bool mpc_acado_params::setReferencePose(const Eigen::Ref<const Eigen::Matrix<float, N_state, 1>> state) {
    acado_reference_states_.block(0, 0, N_state, N_sample) = state.replicate(1, N_sample);

    acado_reference_states_.block(N_state, 0, N_state_cost - N_state, N_sample) =
            Eigen::Matrix<float, N_state_cost - N_state, N_sample>::Zero();

    acado_reference_states_.block(N_state_cost, 0, N_input, N_sample) =
            kHoverInput_.replicate(1, N_sample);

    acado_reference_end_state_.segment(0, N_state) = state;

    acado_reference_end_state_.segment(N_state, N_state_cost - N_state) =
            Eigen::Matrix<float, N_state_cost - N_state, 1>::Zero();

    acado_initializeNodesByForwardSimulation();
    return true;
}

bool
mpc_acado_params::setReferenceTrajectory(const Eigen::Ref<const Eigen::Matrix<float, N_state, N_sample + 1>> states,
                                         const Eigen::Ref<const Eigen::Matrix<float, N_input, N_sample + 1>> inputs) {
    Eigen::Map<Eigen::Matrix<float, N_ref_state, N_sample, Eigen::ColMajor>>
            y(const_cast<float *>(acadoVariables.y));

    acado_reference_states_.block(0, 0, N_state, N_sample) = states.block(0, 0, N_state, N_sample);

    acado_reference_states_.block(N_state, 0, N_state_cost - N_state, N_sample) = Eigen::Matrix<float,
            N_state_cost - N_state, N_sample>::Zero();

    acado_reference_states_.block(N_state_cost, 0, N_input, N_sample) = inputs.block(0, 0, N_input, N_sample);

    acado_reference_end_state_.segment(0, N_state) = states.col(N_sample);
    acado_reference_end_state_.segment(N_state, N_state_cost - N_state) = Eigen::Matrix<float,
            N_state_cost - N_state, 1>::Zero();

    return true;
}

bool mpc_acado_params::update(const Eigen::Ref<const Eigen::Matrix<float, N_state, 1>> state, bool do_preparation) {
    if (!acado_is_prepared_) {
        ROS_WARN("MPC: Solver was triggered without preparation, abort!");
        return false;
    }

    // Check if estimated and reference quaternion live in the same hemisphere.
    acado_initial_state_ = state;
    if (acado_initial_state_.segment(3, 4).dot(
            Eigen::Vector4f(acado_reference_states_.block(3, 0, 4, 1))) < 0.0) {
        acado_initial_state_.segment(3, 4) = -acado_initial_state_.segment(3, 4);
    }

    // Perform feedback step and reset preparation check.
    acado_feedbackStep();
    acado_is_prepared_ = false;

    // Prepare if the solver if wanted
    if (do_preparation) {
        acado_preparationStep();
        acado_is_prepared_ = true;
    }

    return true;
}

bool mpc_acado_params::solve(const Eigen::Ref<const Eigen::Matrix<float, N_state, 1>> state) {
    acado_states_ = state.replicate(1, N_sample + 1);

    acado_inputs_ = kHoverInput_.replicate(1, N_sample);

    return update(state);
}

void QuadrotorMPCController::loadOptimalReplanReferenceData(const std::deque<std::vector<double>> &refPath,
                                                            bool optimal_insert) {
    if (optimal_insert && !refPath.empty()) {
        update_replan_path = false;
        replan_index = 1;
        std::deque<std::vector<double>>().swap(reference_path);
        reference_path = refPath;
//        for (const auto &path: refPath) {
//            reference_path.push_back(path);
//        }
    }
    std::cout << "i" << replan_index << "total num" << reference_path.size() << std::endl;
    if ((ros::Time::now() - replan_delta_time).toSec() > 0.03f && !reference_path.empty()) {
        ff_cmd.position_cmd
                << reference_path[replan_index][0], reference_path[replan_index][1], reference_path[replan_index][2];

        ff_cmd.velocity_cmd
                << reference_path[replan_index][3], reference_path[replan_index][4], reference_path[replan_index][5];
        if (!use_optimal) {
            //TODO 参数传递出现问题，进入不了指点控制
            ff_cmd.acce_cmd
                    << reference_path[replan_index][6], reference_path[replan_index][7], reference_path[replan_index][8];

            ff_cmd.attitude_cmd << 0, 0, 0;
            ff_cmd.rate_cmd << 0, 0, 0;
        } else {
            tf::Quaternion rq;
            geometry_msgs::PoseStamped data_quaternion;
            data_quaternion.pose.orientation.x = reference_path[replan_index][7];
            data_quaternion.pose.orientation.y = reference_path[replan_index][8];
            data_quaternion.pose.orientation.z = reference_path[replan_index][9];
            data_quaternion.pose.orientation.w = reference_path[replan_index][6];

            tf::quaternionMsgToTF(data_quaternion.pose.orientation, rq);
            tf::Matrix3x3(rq).getRPY(ff_cmd.attitude_cmd[0],
                                     ff_cmd.attitude_cmd[1],
                                     ff_cmd.attitude_cmd[2]);
            ff_cmd.rate_cmd
                    << reference_path[replan_index][10], reference_path[replan_index][11], reference_path[replan_index][12];
            ff_cmd.thrust = reference_path[replan_index][13] + reference_path[replan_index][14] +
                            reference_path[replan_index][15] + reference_path[replan_index][16];
            std::cout << reference_path[replan_index][13] << " " << reference_path[replan_index][14] << " "
                      << reference_path[replan_index][15] << " " << reference_path[replan_index][16] << std::endl;
        }
        replan_delta_time = ros::Time::now();
//        if (replan_index == 10) {
//            update_replan_path = true;
//            replan_index = 1;
//        } else {
        replan_index += 1;
//        }
    }


}

void QuadrotorMPCController::loadFeedforwardData() {

    while ((data_index < csv_size_ - 1) && (csv_data_[data_index + 1][0] < current_time_)) {
        data_index += 1;
    }

    Eigen::Vector3d data_position(csv_data_[data_index][1], csv_data_[data_index][2],
                                  csv_data_[data_index][3]);
    Eigen::Vector3d data_velocity(csv_data_[data_index][4], csv_data_[data_index][5], csv_data_[data_index][6]);
    Eigen::Vector3d data_rate(csv_data_[data_index][15], csv_data_[data_index][16], csv_data_[data_index][17]);
    ff_cmd.position_cmd = data_position;
    ff_cmd.velocity_cmd = data_velocity;
    ff_cmd.rate_cmd = data_rate;
    ff_cmd.thrust =
            csv_data_[data_index][11] + csv_data_[data_index][12] + csv_data_[data_index][13] +
            csv_data_[data_index][14];
    tf::Quaternion rq;
    geometry_msgs::PoseStamped data_quaternion;
    data_quaternion.pose.orientation.x = csv_data_[data_index][7];
    data_quaternion.pose.orientation.y = csv_data_[data_index][8];
    data_quaternion.pose.orientation.z = csv_data_[data_index][9];
    data_quaternion.pose.orientation.w = csv_data_[data_index][10];
    tf::quaternionMsgToTF(data_quaternion.pose.orientation, rq);
    tf::Matrix3x3(rq).getRPY(ff_cmd.attitude_cmd[0],
                             ff_cmd.attitude_cmd[1],
                             ff_cmd.attitude_cmd[2]);
    if (data_index == csv_size_ - 1)
        ff_cmd.is_done = 1;
    else
        ff_cmd.is_done = 0;
}

void QuadrotorMPCController::loadLatestData() {
    current_position_ = data_ptr->wrapper_current_position_;
    current_velocity_ = data_ptr->wrapper_current_velocity_;
    current_attitude_ = data_ptr->wrapper_current_attitude_;
    current_orientation = data_ptr->wrapper_current_orientation;
}

QuadrotorMPCController::QuadrotorMPCController(struct DataCentre *wrap_data_ptr) : est_state_(
        (Eigen::Matrix<float, N_state, 1>() <<
                                            0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
                                                                                   reference_states_(
                                                                                           Eigen::Matrix<float, N_state,
                                                                                                   N_sample +
                                                                                                   1>::Zero()),
                                                                                   reference_inputs_(
                                                                                           Eigen::Matrix<float, N_input,
                                                                                                   N_sample +
                                                                                                   1>::Zero()),
                                                                                   predicted_states_(
                                                                                           Eigen::Matrix<float, N_state,
                                                                                                   N_sample +
                                                                                                   1>::Zero()),
                                                                                   predicted_inputs_(
                                                                                           Eigen::Matrix<float, N_input, N_sample>::Zero()) {
    data_ptr = wrap_data_ptr;
    preparation_thread_ = std::thread(&mpc_acado_params::prepare, mpc_wrapper_);
    solve_from_scratch_ = true;

}

bool QuadrotorMPCController::setCurrentState() {
    est_state_(0) = static_cast<float>(current_position_(0));
    est_state_(1) = static_cast<float>(current_position_(1));
    est_state_(2) = static_cast<float>(current_position_(2));
    est_state_(3) = static_cast<float>(current_orientation(3));
    est_state_(4) = static_cast<float>(current_orientation(0));
    est_state_(5) = static_cast<float>(current_orientation(1));
    est_state_(6) = static_cast<float>(current_orientation(2));
    est_state_(7) = static_cast<float>(current_velocity_(0));
    est_state_(8) = static_cast<float>(current_velocity_(1));
    est_state_(9) = static_cast<float>(current_velocity_(2));
}

bool QuadrotorMPCController::setReferenceCSVTrajectory() {
    reference_states_.setZero();
    reference_inputs_.setZero();

    const float dt = 0.03;
    Eigen::Matrix<float, 3, 1> acceleration;
    const Eigen::Matrix<float, 3, 1> gravity(0.0, 0.0, -9.81);
    Eigen::Quaternion<float> q_heading;
    Eigen::Quaternion<float> q_orientation;
    bool quaternion_norm_ok(true);


    while ((data_index < csv_size_ - 1) && (csv_data_[data_index + 1][0] < current_time_)) {
        data_index += 1;
    }
    if (data_index == csv_size_ - 1)
        ff_cmd.is_done = 1;
    else
        ff_cmd.is_done = 0;

    if (data_index + N_sample + 1 > csv_size_) {
        double cur_ind = data_index;
        for (int i = 0; i < csv_size_ - cur_ind; i++) {
            reference_states_.col(i)
                    << static_cast<float>(csv_data_[data_index + i][1]), static_cast<float>(
                    csv_data_[data_index + i][2] ), static_cast<float>(csv_data_[data_index + i][3]),
                    static_cast<float>(csv_data_[data_index + i][10]), static_cast<float>(csv_data_[data_index +
                                                                                                    i][7]), static_cast<float>(csv_data_[
                    data_index + i][8]), static_cast<float>(csv_data_[data_index + i][9]),
                    static_cast<float>(csv_data_[data_index + i][4]), static_cast<float>(csv_data_[data_index +
                                                                                                   i][5]), static_cast<float>(csv_data_[
                    data_index + i][6]);

            reference_inputs_.col(i)
                    << static_cast<float>(csv_data_[data_index + i][11] + csv_data_[data_index + i][12] +
                                          csv_data_[data_index + i][13] +
                                          csv_data_[data_index + i][14]),
                    static_cast<float>(csv_data_[data_index + i][15]), static_cast<float>(csv_data_[data_index +
                                                                                                    i][16]), static_cast<float> (csv_data_[
                    data_index + i][17]);

        }
        for (int i = 0; i < N_sample + 1 - csv_size_ + cur_ind; i++) {
            reference_states_.col(i)
                    << static_cast<float>(csv_data_[csv_size_ - 1][1] ), static_cast<float>(
                    csv_data_[csv_size_ - 1][2] ), static_cast<float>(csv_data_[csv_size_ - 1][3]),
                    static_cast<float>(csv_data_[csv_size_ - 1][10]), static_cast<float>(csv_data_[csv_size_ -
                                                                                                   1][7]), static_cast<float>(csv_data_[
                    csv_size_ - 1][8]), static_cast<float>(csv_data_[csv_size_ - 1][9]),
                    static_cast<float>(csv_data_[csv_size_ - 1][4]), static_cast<float>(csv_data_[csv_size_ -
                                                                                                  1][5]), static_cast<float>(csv_data_[
                    csv_size_ - 1][6]);

            reference_inputs_.col(i)
                    << static_cast<float>(csv_data_[csv_size_ - 1][11] + csv_data_[csv_size_ - 1][12] +
                                          csv_data_[csv_size_ - 1][13] +
                                          csv_data_[csv_size_ - 1][14]),
                    static_cast<float>(csv_data_[csv_size_ - 1][15]), static_cast<float>(csv_data_[csv_size_ -
                                                                                                   1][16]), static_cast<float> (csv_data_[
                    csv_size_ - 1][17]);
        }
    } else {
        for (int i = 0; i < N_sample + 1; i++) {
//        q_heading = Eigen::Quaternion<float>(Eigen::AngleAxis<float>(
//                iterator->heading, Eigen::Matrix<float, 3, 1>::UnitZ()));
//        q_orientation = q_heading * iterator->orientation;
            reference_states_.col(i)
                    << static_cast<float>(csv_data_[data_index + i][1] ), static_cast<float>(
                    csv_data_[data_index + i][2] ), static_cast<float>(csv_data_[data_index + i][3]),
                    static_cast<float>(csv_data_[data_index + i][10]), static_cast<float>(csv_data_[data_index +
                                                                                                    i][7]), static_cast<float>(csv_data_[
                    data_index + i][8]), static_cast<float>(csv_data_[data_index + i][9]),
                    static_cast<float>(csv_data_[data_index + i][4]), static_cast<float>(csv_data_[data_index +
                                                                                                   i][5]), static_cast<float>(csv_data_[
                    data_index + i][6]);

            reference_inputs_.col(i)
                    << static_cast<float>(csv_data_[data_index + i][11] + csv_data_[data_index + i][12] +
                                          csv_data_[data_index + i][13] +
                                          csv_data_[data_index + i][14]),
                    static_cast<float>(csv_data_[data_index + i][15]), static_cast<float>(csv_data_[data_index +
                                                                                                    i][16]), static_cast<float> (csv_data_[
                    data_index + i][17]);

        }
    }
//    }
    return true;
}

bool QuadrotorMPCController::MPCControl() {
    ros::Time call_time = ros::Time::now();
    const clock_t start = clock();
    preparation_thread_.join();

    // Convert everything into Eigen format.
    setCurrentState();
//    setReferenceTrajectory();
    setReferenceCSVTrajectory();
    static const bool do_preparation_step(false);

    // Get the feedback from MPC.
    mpc_wrapper_.setReferenceTrajectory(reference_states_, reference_inputs_);
    if (solve_from_scratch_) {
        ROS_INFO("Solving MPC with hover as initial guess.");
        mpc_wrapper_.solve(est_state_);
        solve_from_scratch_ = false;
    } else {
        mpc_wrapper_.update(est_state_, do_preparation_step);
    }
    mpc_wrapper_.getStates(predicted_states_);
    mpc_wrapper_.getInputs(predicted_inputs_);

    // Publish the predicted trajectory.
//    publishPrediction(predicted_states_, predicted_inputs_, call_time);

    // Start a thread to prepare for the next execution.
    preparation_thread_ = std::thread(&QuadrotorMPCController::preparationThread, this);

    // Timing
    const clock_t end = clock();
    timing_feedback_ = 0.9 * timing_feedback_ +
                       0.1 * double(end - start) / CLOCKS_PER_SEC;
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

    // Return the input control command.
    thrust_attitude_cmd_.thrust = predicted_inputs_.col(0)[0] * thrust_ave_ / 9.81;
    thrust_attitude_cmd_.body_rate.x = predicted_inputs_.col(0)[1];
    thrust_attitude_cmd_.body_rate.y = predicted_inputs_.col(0)[2];
    thrust_attitude_cmd_.body_rate.z = predicted_inputs_.col(0)[3];
    thrust_attitude_cmd_.type_mask = 128;

    data_ptr->thrust_attitude_cmd_ = thrust_attitude_cmd_;

//    data_ptr->pub_acc_setpoint.vector.x = acc_des[0];
//    data_ptr->pub_acc_setpoint.vector.y = acc_des[1];
//    data_ptr->pub_acc_setpoint.vector.z = acc_des[2];
//    data_ptr->pub_acc_setpoint.header.stamp = ros::Time::now();
//    data_ptr->pub_setpoint_position_.header.frame_id = "odom";

    data_ptr->pub_setpoint_position_.pose.position.x = ff_cmd.position_cmd[0];
    data_ptr->pub_setpoint_position_.pose.position.y = ff_cmd.position_cmd[1];
    data_ptr->pub_setpoint_position_.pose.position.z = ff_cmd.position_cmd[2];
    data_ptr->pub_setpoint_position_.header.stamp = ros::Time::now();
    data_ptr->pub_setpoint_position_.header.frame_id = "odom";

    data_ptr->pub_setpoint_velocity_.twist.linear.x = ff_cmd.velocity_cmd[0];;
    data_ptr->pub_setpoint_velocity_.twist.linear.y = ff_cmd.velocity_cmd[1];
    data_ptr->pub_setpoint_velocity_.twist.linear.z = ff_cmd.velocity_cmd[2];
    data_ptr->pub_setpoint_velocity_.header.stamp = ros::Time::now();
    data_ptr->pub_setpoint_velocity_.header.frame_id = "odom";

    data_ptr->pub_euler_attitude_.vector.x = current_attitude_[0] * 180 / PI;
    data_ptr->pub_euler_attitude_.vector.y = current_attitude_[1] * 180 / PI;
    data_ptr->pub_euler_attitude_.vector.z = current_attitude_[2] * 180 / PI;
    data_ptr->pub_euler_attitude_.header.stamp = ros::Time::now();

    tf::Quaternion rq;
    double phi_des, theta_des, psi_des;
    geometry_msgs::PoseStamped data_quaternion;
    data_quaternion.pose.orientation.x = predicted_states_.col(0)[4];
    data_quaternion.pose.orientation.y = predicted_states_.col(0)[5];
    data_quaternion.pose.orientation.z = predicted_states_.col(0)[6];
    data_quaternion.pose.orientation.w = predicted_states_.col(0)[3];
    tf::quaternionMsgToTF(data_quaternion.pose.orientation, rq);
    tf::Matrix3x3(rq).getRPY(phi_des, theta_des, psi_des);
    data_ptr->pub_setpoint_attitude_.vector.x = ff_cmd.attitude_cmd[0] * 180 / PI;
    data_ptr->pub_setpoint_attitude_.vector.y = ff_cmd.attitude_cmd[1] * 180 / PI;
    data_ptr->pub_setpoint_attitude_.vector.z = ff_cmd.attitude_cmd[2] * 180 / PI;
    data_ptr->pub_setpoint_attitude_.header.stamp = ros::Time::now();


    data_ptr->ref_path_point.pose.position.x = ff_cmd.position_cmd[0];
    data_ptr->ref_path_point.pose.position.y = ff_cmd.position_cmd[1];
    data_ptr->ref_path_point.pose.position.z = ff_cmd.position_cmd[2];
    data_ptr->ref_path_point.header.stamp = ros::Time::now();
    data_ptr->ref_path_point.header.frame_id = "odom";

    data_ptr->fly_path_point.pose.position.x = current_position_[0];
    data_ptr->fly_path_point.pose.position.y = current_position_[1];
    data_ptr->fly_path_point.pose.position.z = current_position_[2];
    data_ptr->fly_path_point.header.stamp = ros::Time::now();
    data_ptr->fly_path_point.header.frame_id = "odom";
}

void QuadrotorMPCController::readCsvData(std::string dataset) {
    std::ifstream fin(dataset);
    std::string line, word;
    std::vector<std::string> row;
    std::vector<std::vector<std::string>> content;
    int row_num = 0, col_num = 0;

    while (getline(fin, line)) {
        row.clear();
        std::stringstream str(line);
        col_num = 0;
        while (getline(str, word, ',')) {
            row.push_back(word);
            content.push_back(row);
            col_num += 1;
        }
        row_num += 1;
    }

    std::vector<std::string> rad[18];
    for (int i = col_num * 3 - 1; i < content.size(); i += col_num) {
        if (stof(content[i][col_num - 3]) < 0.001) {
            ff_cmd.is_done = 1;
            break;
        }
        rad[0].push_back(std::to_string(0.03 * rad[0].size()));
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
    }

    //逐渐衰减
    int vel_decent = floor(rad[0].size() * 0.1);
//    std::cout<<vel_decent<<std::endl;
    std::vector<float> decent_vector;
    for (int i = 0; i < rad[0].size(); i++) {
        if (i + vel_decent > rad[0].size()) {
            decent_vector.push_back((1 / float(vel_decent)) * (rad[0].size() - i - 1));
        } else {
            decent_vector.push_back(1);
        }
    }
    for (auto item: decent_vector) {
//        std::cout<<item<<std::endl;
    }
    for (int i = 0; i < rad[0].size(); i++) {
        std::string px = rad[11][i];
        std::string py = rad[12][i];
        rad[11][i] = std::to_string(stof(px) * decent_vector[i]);
        rad[12][i] = std::to_string(stof(py) * decent_vector[i]);
        std::string x = rad[4][i];
        std::string y = rad[5][i];
        rad[4][i] = std::to_string(stof(x) * decent_vector[i]);
        rad[5][i] = std::to_string(stof(y) * decent_vector[i]);


//
////        string wx = rad[7][i];
////        string wy = rad[8][i];
////        string wz = rad[9][i];
////        rad[7][i] = std::to_string(stof(wx) * decent_vector[i]);
////        rad[8][i] = std::to_string(stof(wx) * decent_vector[i]);
////        rad[9][i] = std::to_string(stof(wz) * decent_vector[i]);
    }
    //逐渐减小

// 提前中止
//double total_num = rad[0].size();
//  std::cout<<"total_num"<<total_num<<std::endl;
//for(int i = 0;i< total_num;i++){
//    if(i > (total_num * 0.8)){
//        rad[0].pop_back();
//        rad[1].pop_back();
//        rad[2].pop_back();
//        rad[3].pop_back();
//        rad[4].pop_back();
//        rad[5].pop_back();
//        rad[6].pop_back();
//        rad[7].pop_back();
//        rad[8].pop_back();
//        rad[9].pop_back();
//        rad[10].pop_back();
//        rad[11].pop_back();
//        rad[12].pop_back();
//        rad[13].pop_back();
//        rad[14].pop_back();
//        rad[15].pop_back();
//        rad[16].pop_back();
//        rad[17].pop_back();}
//}
//提前终止


    csv_size_ = rad[0].size();
    for (int k = 0; k < rad[0].size(); k++) {
        for (int j = 0; j < 18; j++) {
            csv_data_[k][j] = stof(rad[j][k]);
            std::cout << csv_data_[k][j] << " ";
        }
//    cout << endl;
    }
    fin.close();
}

void QuadrotorMPCController::preparationThread() {
    const clock_t start = clock();

    mpc_wrapper_.prepare();

    // Timing
    const clock_t end = clock();
    timing_preparation_ = 0.9 * timing_preparation_ +
                          0.1 * double(end - start) / CLOCKS_PER_SEC;
}

QuadrotorMPCController::~QuadrotorMPCController() {

}
