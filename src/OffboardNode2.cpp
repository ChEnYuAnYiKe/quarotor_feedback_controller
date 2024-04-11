#include "OffboardWrapper.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb2_node");
    // ros::NodeHandle nh;
    geometry_msgs::PoseStamped g_position_setpoint0, g_position_setpoint1;
    g_position_setpoint0.pose.position.x = -2.3;
    g_position_setpoint0.pose.position.y = -1;
    g_position_setpoint0.pose.position.z = 1.5;
    g_position_setpoint1.pose.position.x = -2.3;
    g_position_setpoint1.pose.position.y = 1;
    g_position_setpoint1.pose.position.z = 1.5;
    // OffboardWrapper wrapper0(g_position_setpoint, "/uav0");
    // wrapper0.run();
    OffboardWrapper wrapper(g_position_setpoint0, g_position_setpoint1,"/uav2", "/offb2_node",
                            "/home/zhoujin/zj/src/quarotor_feedback_controller/library/result2.csv");
    wrapper.run();

    return 0;
}

