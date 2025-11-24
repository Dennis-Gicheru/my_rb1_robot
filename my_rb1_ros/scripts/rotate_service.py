#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from my_rb1_ros.srv import Rotate, RotateResponse

class RotateService(object):
    def __init__(self):
        rospy.init_node("rotate_service_server")
        rospy.loginfo("Rotate Service Ready")

        self.current_yaw = 0.0
        self.odom_received = False

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.service = rospy.Service('/rotate_robot', Rotate, self.handle_rotate)
        rospy.spin()

    def odom_callback(self, msg):
        
        ori = msg.pose.pose.orientation
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        (_, _, yaw) = euler_from_quaternion(quaternion)

        self.current_yaw = yaw
        self.odom_received = True

    def handle_rotate(self, req):
        rospy.loginfo("Service Requested: rotate {} degrees".format(req.degrees))

        while not self.odom_received:
            rospy.sleep(0.05)

        # Convert degrees to radians
        target_angle = math.radians(req.degrees)

        
        start_yaw = self.current_yaw
        final_yaw = self.normalize_angle(start_yaw + target_angle)

        rate = rospy.Rate(50)
        twist = Twist()

        # Rotation speed
        speed = 0.3 if req.degrees > 0 else -0.3

        while not rospy.is_shutdown():
            error = self.normalize_angle(final_yaw - self.current_yaw)

            if abs(error) < math.radians(2): 
                break

            twist.angular.z = speed
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop the robot
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        rospy.loginfo("Service Completed")

        return RotateResponse("Rotation of {} degrees completed".format(req.degrees))

    def normalize_angle(self, angle):
        """Keeps angle in range [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))


if __name__ == "__main__":
    RotateService()
