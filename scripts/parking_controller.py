#!/usr/bin/env python

from math import sqrt
import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        # Calculate the distance from the car to the cone
        distance = sqrt(self.relative_x ** 2 + self.relative_y ** 2)

        # Calculate the angle between the car's orientation and the cone
        angle = np.arctan2(self.relative_y, self.relative_x)

        # Set desired angle and velocity for parking
        desired_angle = 0.0  # radians (adjust as needed)
        desired_velocity = 0.3  # meters/sec (adjust as needed)

        # Calculate errors
        angle_error = desired_angle - angle
        distance_error = self.parking_distance - distance

        # Define control gains
        angle_gain = 1.0
        distance_gain = 0.5

        # Calculate control signals
        steering_angle = angle_gain * angle_error
        velocity = distance_gain * distance_error

        # Limit velocity
        if velocity > desired_velocity:
            velocity = desired_velocity

        # Publish desired steering angle and velocity
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = velocity
        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################
        error_msg.relative_x = self.relative_x
        error_msg.relative_y = self.relative_y
        error_msg.distance = sqrt(self.relative_x ** 2 + self.relative_y ** 2)
        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
