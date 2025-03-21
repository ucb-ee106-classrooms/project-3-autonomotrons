# u : list
#     A list of system inputs, where, for the ith data point u[i],
#     u[i][0] is timestamp (s),
#     u[i][1] is left wheel rotational speed (rad/s), and
#     u[i][2] is right wheel rotational speed (rad/s).
# x : list
#     A list of system states, where, for the ith data point x[i],
#     x[i][0] is timestamp (s),
#     x[i][1] is bearing (rad),
#     x[i][2] is translational position in x (m),
#     x[i][3] is translational position in y (m),
#     x[i][4] is left wheel rotational position (rad), and
#     x[i][5] is right wheel rotational position (rad).
# y : list
#     A list of system outputs, where, for the ith data point y[i],
#     y[i][0] is timestamp (s),
#     y[i][1] is translational position in x (m) when freeze_bearing:=true,
#     y[i][1] is distance to the landmark (m) when freeze_bearing:=false,
#     y[i][2] is translational position in y (m) when freeze_bearing:=true, and
#     y[i][2] is relative bearing (rad) w.r.t. the landmark when
#     freeze_bearing:=false.

# we need to convert cmd vel messages to left and right angular velocity for some reason
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import time
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
        self.last_u = None
        self.last_x = None
        self.last_y = None

    def odom_callback(self, msg):
        point = msg.pose.pose.position
        phi = msg.pose.pose.orientation.z
        if not self.x:
            #self.x.append([rospy.get_time(), phi, point[0], point[1], 0, 0])
            marray = Float64MultiArray()
            marray.data = [time.perf_counter(), phi, point.x, point.y, 0, 0]
            self.last_x = marray
            #self.x_pub.publish(marray)
        marray = Float64MultiArray()
        marray.data = [time.perf_counter(), phi, point.x, point.y]
        self.last_y = marray
        #self.y_pub.publish(marray)

    def twist_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        right_wheel_vel = (angular_vel * self.d) + linear_vel
        left_wheel_vel = - (angular_vel * self.d) + linear_vel
        marray = Float64MultiArray()
        marray.data = [left_wheel_vel, right_wheel_vel]
        self.last_u = marray
        print("LAST U :", self.last_u)
        #self.u_pub.publish(marray)


    def publish_states(self, event):
        """Publish the last states to the topics."""
        if self.last_u is not None:
            self.u_pub.publish(self.last_u)
        
        if self.last_x is not None:
            self.x_pub.publish(self.last_x)
        
        if self.last_y is not None:
            self.y_pub.publish(self.last_y)

def main():
    """Entry point of the estimator node.

    Returns
    -------
        None
    """
    ekf = turtlebot_ekf()
    rospy.spin()
    rospy.shutdown()

if __name__ == '__main__':
    main()
