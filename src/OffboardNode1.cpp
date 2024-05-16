#include "OffboardWrapper.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb1_node");
    ros::NodeHandle nh1;
    geometry_msgs::PoseStamped g_position_setpoint0, g_position_setpoint1;

	nh1.param("position1_x", g_position_setpoint0.pose.position.x, 3.);
    nh1.param("position1_y", g_position_setpoint0.pose.position.y, 3.);
    nh1.param("position1_z", g_position_setpoint0.pose.position.z, 1.5);
    nh1.param("position2_x", g_position_setpoint1.pose.position.x, 3.);
    nh1.param("position2_y", g_position_setpoint1.pose.position.y, 3.);
    nh1.param("position2_z", g_position_setpoint1.pose.position.z, 1.5);

    // g_position_setpoint0.pose.position.x = 3;
    // g_position_setpoint0.pose.position.y = 3;
    // g_position_setpoint0.pose.position.z = 1.5;
    // g_position_setpoint1.pose.position.x = 3;
    // g_position_setpoint1.pose.position.y = 3;
    // g_position_setpoint1.pose.position.z = 1.5;

	// OffboardWrapper wrapper(g_position_setpoint0, "/uav0", "/offb_node", "/home/zhoujin/time_optimal_trajectory/example/result.csv");
    OffboardWrapper wrapper(g_position_setpoint0, g_position_setpoint1,"", "/offb1_node",
                            "/home/zhoujin/zj/src/quarotor_feedback_controller/library/result.csv");
    wrapper.run();
    // OffboardWrapper wrapper(g_position_setpoint1, "/uav1");
    // wrapper.run();

    return 0;
}

