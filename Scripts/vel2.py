import numpy as np
import rospy
import threading
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
from time import time


def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler


def thread_job():
    rospy.spin()


class VelocityBody:

    def __init__(self):
        self.euler = Vector3Stamped()
        rospy.Subscriber("/vrpn_client_node/jiahao2/pose", PoseStamped, self.posCb)
        # self.pos_pub = rospy.Publisher("mavros/local_position/velocity", TwistStamped, queue_size = 1)# topic 
        # self.pos_pub = rospy.Publisher("mavros/setpoint_accel/accel", Vector3Stamped, queue_size = 1)# topic 
        self.vel_pub = rospy.Publisher("uav2/outer_velocity", TwistStamped, queue_size=10)  # topic
        self.acc_pub = rospy.Publisher("uav2/outer_acc", TwistStamped, queue_size=10)  # topic
        self.pos_pub = rospy.Publisher("uav2/outer_position", PoseStamped, queue_size=10)
        self.vel_error_pub = rospy.Publisher("uav2/outer_error", TwistStamped, queue_size=10)
        self.add_thread = threading.Thread(target=thread_job)
        self.add_thread.start()

        self.current_position = PoseStamped()
        self.last_position = PoseStamped()
        self.last_velocity = PoseStamped()
        self.last_velocity_est = TwistStamped()

    def posCb(self, msg):
        self.current_position = msg



def main():
    print("start!")
    rospy.init_node('outer_velocity2', anonymous=True)

    cnt = VelocityBody()
    rate = rospy.Rate(100)
    outer_velocity = TwistStamped()
    outer_acc = TwistStamped()
    # ROS main loop
    last_time = time()
    kx = 0.5
    ky = 0.5
    kz = 0.5
    last_queue_x = []
    last_queue_y = []
    last_queue_z = []
    last_pos_x = []
    last_pos_y = []
    last_pos_z = []
    last_vel_x = 0
    last_vel_y = 0
    last_vel_z = 0
    while not rospy.is_shutdown():
        current_time = time()
        deltat = current_time - last_time
        last_pos_x.append(cnt.current_position.pose.position.x)
        last_pos_y.append(cnt.current_position.pose.position.y)
        last_pos_z.append(cnt.current_position.pose.position.z)
        print("pos x:", cnt.current_position.pose.position.x)
        if (len(last_pos_x) > 5):
            last_pos_x.pop(0)
            last_pos_y.pop(0)
            last_pos_z.pop(0)
        pose_est_x = np.sum(np.array(last_pos_x)) / 5
        pose_est_y = np.sum(np.array(last_pos_y)) / 5
        pose_est_z = np.sum(np.array(last_pos_z)) / 5
        last_pos_x = list(last_pos_x)
        last_pos_y = list(last_pos_y)
        last_pos_z = list(last_pos_z)
        pos_est_pub = PoseStamped()
        pos_est_pub.pose.position.x = pose_est_x
        pos_est_pub.pose.position.y = pose_est_y
        pos_est_pub.pose.position.z = pose_est_z
        pos_est_pub.header.stamp = rospy.Time.now()
        cnt.pos_pub.publish(pos_est_pub)
        # pose_est_x = cnt.current_position.pose.position.x * (1 - kx) + kx * cnt.last_position.pose.position.x
        # pose_est_y = cnt.current_position.pose.position.y * (1 - ky) + ky * cnt.last_position.pose.position.y
        # pose_est_z = cnt.current_position.pose.position.z * (1 - kz) + kz * cnt.last_position.pose.position.z
        vel_x_cur = (pose_est_x - cnt.last_position.pose.position.x) / deltat
        vel_y_cur = (pose_est_y - cnt.last_position.pose.position.y) / deltat
        vel_z_cur = (pose_est_z - cnt.last_position.pose.position.z) / deltat
        last_queue_x.append(vel_x_cur)
        last_queue_y.append(vel_y_cur)
        last_queue_z.append(vel_z_cur)
        if (len(last_queue_x) > 5):
            last_queue_x.pop(0)
            last_queue_y.pop(0)
            last_queue_z.pop(0)
        outer_velocity.twist.linear.x = np.sum(np.array(last_queue_x)) / 5
        outer_velocity.twist.linear.y = np.sum(np.array(last_queue_y)) / 5
        outer_velocity.twist.linear.z = np.sum(np.array(last_queue_z)) / 5
        vel_error = TwistStamped()
        vel_error.twist.linear.x = abs(vel_x_cur - cnt.last_velocity_est.twist.linear.x)
        vel_error.twist.linear.y = abs(vel_y_cur - cnt.last_velocity_est.twist.linear.y)
        vel_error.twist.linear.z = abs(vel_z_cur - cnt.last_velocity_est.twist.linear.z)
        vel_error.header.stamp = rospy.Time.now()
        cnt.vel_error_pub.publish(vel_error)
        if (abs(vel_x_cur - cnt.last_velocity_est.twist.linear.x) > 0.1):
            outer_velocity.twist.linear.x = cnt.last_velocity_est.twist.linear.x
        if (abs(vel_y_cur - cnt.last_velocity_est.twist.linear.y) > 0.1):
            outer_velocity.twist.linear.y = cnt.last_velocity_est.twist.linear.y
        if (abs(vel_z_cur - cnt.last_velocity_est.twist.linear.z) > 0.1):
            outer_velocity.twist.linear.z = cnt.last_velocity_est.twist.linear.z
        last_queue_x = list(last_queue_x)
        last_queue_y = list(last_queue_y)
        last_queue_z = list(last_queue_z)
        # outer_velocity.twist.linear.x = outer_velocity.twist.linear.x * (1 - kx) + (cnt.current_position.pose.position.x - cnt.last_position.pose.position.x) * kx / deltat
        # outer_velocity.twist.linear.y = outer_velocity.twist.linear.y * (1 - ky) + (cnt.current_position.pose.position.y - cnt.last_position.pose.position.y) * ky / deltat
        # outer_velocity.twist.linear.z = outer_velocity.twist.linear.z * (1 - kz) + (cnt.current_position.pose.position.z - cnt.last_position.pose.position.z) * kz / deltat
        outer_velocity.header.stamp = rospy.Time.now()
        cnt.vel_pub.publish(outer_velocity)

        outer_acc.twist.linear.x = (outer_velocity.twist.linear.x - last_vel_x) / deltat
        outer_acc.twist.linear.y = (outer_velocity.twist.linear.y - last_vel_y) / deltat
        outer_acc.twist.linear.z = (outer_velocity.twist.linear.z - last_vel_z) / deltat + 9.81
        outer_acc.header.stamp = rospy.Time.now()
        cnt.acc_pub.publish(outer_acc)
        last_vel_x = outer_velocity.twist.linear.x
        last_vel_y = outer_velocity.twist.linear.y
        last_vel_z = outer_velocity.twist.linear.z

        cnt.last_velocity.pose.position.x = vel_x_cur
        cnt.last_velocity.pose.position.y = vel_y_cur
        cnt.last_velocity.pose.position.z = vel_z_cur
        cnt.last_velocity_est = outer_velocity
        cnt.last_position.pose.position.x = pose_est_x
        cnt.last_position.pose.position.y = pose_est_y
        cnt.last_position.pose.position.z = pose_est_z

        last_time = current_time
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
