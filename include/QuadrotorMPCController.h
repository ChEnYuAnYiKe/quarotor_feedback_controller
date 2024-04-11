//
// Created by zfg on 23-5-5.
//
#pragma once

#include <thread>
#include "QuadrotorFeedbackController.h"
#include "acado_auxiliary_functions.h"
#include "acado_common.h"

const static int N_sample = ACADO_N;      // number of samples
const static int N_state = ACADO_NX;   // number of states
const static int N_ref_state = ACADO_NY;     // number of reference states
const static int N_end_ref_state = ACADO_NYN; // number of end reference states
const static int N_input = ACADO_NU;   // number of inputs
const static int N_state_cost = ACADO_NY - ACADO_NU; // number of state costs
const static int N_online_data = ACADO_NOD;     // number of online data

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

class mpc_acado_params {
public:
    void getState(const int node_index, Eigen::Ref<Eigen::Matrix<float, N_state, 1>> return_state);

    void getStates(Eigen::Ref<Eigen::Matrix<float, N_state, N_sample + 1>> return_states);

    void getInput(const int node_index, Eigen::Ref<Eigen::Matrix<float, N_input, 1>> return_input);

    void getInputs(Eigen::Ref<Eigen::Matrix<float, N_input, N_sample>> return_input);

    mpc_acado_params();

    bool prepare();

    bool setReferencePose(Eigen::Ref<const Eigen::Matrix<float, N_state, 1>> state);

    bool setReferenceTrajectory(
            Eigen::Ref<const Eigen::Matrix<float, N_state, N_sample + 1>> states,
            Eigen::Ref<const Eigen::Matrix<float, N_input, N_sample + 1>> inputs);

    bool solve(Eigen::Ref<const Eigen::Matrix<float, N_state, 1>> state);

    bool update(Eigen::Ref<const Eigen::Matrix<float, N_state, 1>> state, bool do_preparation = true);

private:

    Eigen::Map<Eigen::Matrix<float, N_ref_state, N_sample, Eigen::ColMajor>>
            acado_reference_states_{acadoVariables.y};

    Eigen::Map<Eigen::Matrix<float, N_end_ref_state, 1, Eigen::ColMajor>>
            acado_reference_end_state_{acadoVariables.yN};

    Eigen::Map<Eigen::Matrix<float, N_state, 1, Eigen::ColMajor>>
            acado_initial_state_{acadoVariables.x0};

    Eigen::Map<Eigen::Matrix<float, N_state, N_sample + 1, Eigen::ColMajor>>
            acado_states_{acadoVariables.x};

    Eigen::Map<Eigen::Matrix<float, N_input, N_sample, Eigen::ColMajor>>
            acado_inputs_{acadoVariables.u};

    Eigen::Map<Eigen::Matrix<float, N_online_data, N_sample + 1, Eigen::ColMajor>>
            acado_online_data_{acadoVariables.od};

    Eigen::Map<Eigen::Matrix<float, N_ref_state, N_ref_state * N_sample>>
            acado_W_{acadoVariables.W};

    Eigen::Map<Eigen::Matrix<float, N_end_ref_state, N_end_ref_state>>
            acado_W_end_{acadoVariables.WN};

    Eigen::Map<Eigen::Matrix<float, 4, N_sample, Eigen::ColMajor>>
            acado_lower_bounds_{acadoVariables.lbValues};

    Eigen::Map<Eigen::Matrix<float, 4, N_sample, Eigen::ColMajor>>
            acado_upper_bounds_{acadoVariables.ubValues};

    Eigen::Matrix<float, N_ref_state, N_ref_state> W_ = (Eigen::Matrix<float, N_ref_state, 1>() <<
                                                                                                200, 200, 500,
            50 * Eigen::Matrix<float, 4, 1>::Ones(),
            10 * Eigen::Matrix<float, 3, 1>::Ones(),
            1, 1, 1, 1).finished().asDiagonal();

    Eigen::Matrix<float, N_end_ref_state, N_end_ref_state> WN_ =
            W_.block(0, 0, N_end_ref_state, N_end_ref_state);
    const float dt = 0.1;
    bool acado_is_prepared_ = false;
    const Eigen::Matrix<real_t, N_input, 1> kHoverInput_ = (Eigen::Matrix<real_t, N_input, 1>()
            << 9.81, 0.0, 0.0, 0.0).finished();

};

class QuadrotorMPCController {
private:
    bool use_optimal = true;
    int data_index;
    int csv_size_;
    double csv_data_[20000][18];
    // std::string current_state_;

    Eigen::Vector3d current_position_, current_velocity_, current_attitude_, current_angular_acc;
    Eigen::Vector3d position_error_sum_, velocity_error_sum_;
    Eigen::Vector4d current_orientation;
    mavros_msgs::AttitudeTarget thrust_attitude_cmd_;


public:
    struct DataCentre *data_ptr;
    struct FeedforwardCmd {
        Eigen::Vector3d position_cmd, velocity_cmd, acce_cmd, attitude_cmd, rate_cmd;
        double thrust;
        bool is_done;
    } ff_cmd;

    void readCsvData(std::string dataset);

    mpc_acado_params mpc_wrapper_;
    std::thread preparation_thread_;
    float timing_feedback_, timing_preparation_;
    bool solve_from_scratch_;
    Eigen::Matrix<float, N_state, 1> est_state_;
    Eigen::Matrix<float, N_state, N_sample + 1> reference_states_;
    Eigen::Matrix<float, N_input, N_sample + 1> reference_inputs_;
    Eigen::Matrix<float, N_state, N_sample + 1> predicted_states_;
    Eigen::Matrix<float, N_input, N_sample> predicted_inputs_;
    Eigen::Matrix<float, 3, 1> point_of_interest_;

    bool setCurrentState();

    bool setReferenceTrajectory();

    bool setReferenceCSVTrajectory();

    bool MPCControl();

    void preparationThread();

    std::deque<std::vector<double>> reference_path;
    bool update_replan_path = true;
    int replan_index = 1;
    ros::Time replan_delta_time;
    double current_time_;
    double thrust_ave_;

    void loadLatestData();

    void loadFeedforwardData();

    void loadOptimalReplanReferenceData(const std::deque<std::vector<double>> &refPath, bool optimal_insert);

    QuadrotorMPCController(struct DataCentre *wrap_data_ptr);

    ~QuadrotorMPCController();

};

