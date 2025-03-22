import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import time
import math

class turtlebot_ekf():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.u = []
        self.x = []
        self.y = []
        self.d = 0.08
        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
        self.sub_state = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.x_pub = rospy.Publisher("x", Float64MultiArray, queue_size = 10)
        self.y_pub = rospy.Publisher("y", Float64MultiArray, queue_size = 10)
        self.u_pub = rospy.Publisher("u", Float64MultiArray, queue_size = 10)
        timer = rospy.Timer(rospy.Duration(0.1), self.publish_states)
        self.u = [0, 0]
        self.last_u = None
        self.last_x = None
        self.last_y = None

    def odom_callback(self, msg):
        point = msg.pose.pose.position
        qX = msg.pose.pose.orientation.x
        qW = msg.pose.pose.orientation.w
        qY = msg.pose.pose.orientation.y
        qZ = msg.pose.pose.orientation.z
        phi = self.quat_to_yaw(qW, qX, qY, qZ)
        if not self.x:
            measured_state = np.array([phi, point.x, point.y])
            marray = Float64MultiArray()
            marray.data = [time.perf_counter()] + measured_state.flatten().tolist() + [self.u[0], self.u[1]]
            self.last_x = marray
            self.x_pub.publish(marray)
        marray = Float64MultiArray()
        noisy_state = np.random.multivariate_normal(measured_state, np.array([[1, 0.5, 0.1], [0.5, 0.1, 0.2], [0.1, 0.2, 0.2]])*0.0001)
        marray.data = [time.perf_counter()] + (noisy_state).flatten().tolist()
        self.last_y = marray

    def twist_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        right_wheel_vel = (angular_vel * self.d) + linear_vel
        left_wheel_vel = - (angular_vel * self.d) + linear_vel
        marray = Float64MultiArray()
        marray.data = [left_wheel_vel, right_wheel_vel]
        self.u[0] = left_wheel_vel
        self.u[1] = right_wheel_vel
        self.last_u = marray


    def publish_states(self, event):
        if self.last_u is not None:
            self.u_pub.publish(self.last_u)
        
        if self.last_x is not None:
            self.x_pub.publish(self.last_x)
        
        if self.last_y is not None:
            self.y_pub.publish(self.last_y)
    
    def quat_to_yaw(self, q0, q1, q2, q3):
        # NOTE: q0, q1, q2, q3 -> w, x, y, z 
        return math.atan2(2.0 * (q3 * q0) + (q1 * q2), -1.0 + 2.0 * (q0 * q0 + q1 * q1))

def main():
    ekf = turtlebot_ekf()
    rospy.spin()
    rospy.shutdown()

if __name__ == '__main__':
    main()
