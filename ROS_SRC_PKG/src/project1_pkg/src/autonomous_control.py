#!/usr/bin/env python3
### Project 1: Autonomous (Closed-Loop) Proportional Controller for the Truck and Trailer ###

##-----------------------------------------------------------------------------##
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray


import numpy as np
from tf_transformations import euler_from_quaternion


class Autonomous(Node):

    def __init__(self):
        super().__init__('autonomous_control')

        # Subscriber to retrieve localisation information of the Truck "dummy_link" (Topic: /odom)
        self.odom_sub = self.create_subscription(PoseStamped,
                                                '/odom',
                                                self.listener_callback,
                                                10)
        self.odom_sub # Prevent unused variable warning

        # Publisher to publish velocity data the Truck Rear Wheels(Topic: /velocity_controller/commands)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Publisher to publish position data the Truck Front Joints(Topic: /position_controller/commands)
        self.pos_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Wheel Velocity, Joint Position, and Destination
        self.velocity = 0.0
        self.position = 0.0
        self.destination = [10.0, 10.0]

        # # Proportional Gains
        self.vel_gain = 3.0
        self.pos_gain = 1.0

        # Define Max. velocities and Joint angle position
        self.max_vel = 100
        self.max_pos = 0.30

        self.i = 1
        

    # Function to publish wheel velocity and joint position whenever the subscriber hears from '/odom'
    def vel_pos_publisher(self, vel, pos):
        # Velocity message to publish to the Truck
        vel_msg = Float64MultiArray()
        # Constant velocity v = 15
        vel_msg.data = [vel, vel] # Rear-left and Rear-right Wheels
        vel_msg.layout.dim = []
        vel_msg.layout.data_offset = 1

        # Position message to publish to the Truck
        pos_msg = Float64MultiArray()
        # Constant angle a = 1.0  Range: {-3.0, 3.0}
        pos_msg.data = [pos, pos] # Front-left and Front-right joints
        pos_msg.layout.dim = []
        pos_msg.layout.data_offset = 1

        # Publish message
        self.vel_pub.publish(vel_msg)
        self.pos_pub.publish(pos_msg)

        # self.get_logger().info(f'Publish[{self.i}] velocity: {vel_msg.data}, position: {pos_msg.data}')
        self.i += 1

    # Call back to msg retrieved from '/odom' topic 
    def listener_callback(self, msg):
        # Obtain the Coordinates (X, Y) of the Truck "dummy_link"
        X = msg.pose.position.x
        Y = msg.pose.position.y

        # Distance of the Truck from the Destination
        dist = np.sqrt((self.destination[0] - X)**2 + (self.destination[1] - Y)**2)

        # Slope between the Truck and the Destination
        if dist == 0.0:
            theta = 0.0
        else:
            theta = np.degrees(np.arcsin((10.0 - Y) / dist))
            if  X > self.destination[0]:
                if Y <= self.destination[1]:
                    theta += 90.0
                else:
                    theta += -90.0
            theta = np.radians(theta)


        # Obtain the Quaternion Orientation (X, Y, Z, W) of the Truck "dummy_link"
        X_quat = msg.pose.orientation.x
        Y_quat = msg.pose.orientation.y
        Z_quat = msg.pose.orientation.z
        W_quat = msg.pose.orientation.w

        # Euler Orientation (in radians) from the Quaternions
        [roll, pitch, yaw] = euler_from_quaternion([X_quat, Y_quat, Z_quat, W_quat])
        
        # self.get_logger().info(f'Distance: {dist}, theta: {theta}, yaw: {yaw}')

        ## Define a proportional Controller for Wheel Velocity and Joint Position ##
        
        # Wheel Velocity
        self.velocity = self.vel_gain * dist
        if self.velocity > 0.0:
            self.velocity = min(self.velocity, self.max_vel)
        
        # Joint Position
        self.position = self.pos_gain * (yaw - theta) # yaw: orientation of the truck (in radians)
        if self.position > 0.0:
            self.position = min(self.position, self.max_pos)
        elif self.position < 0.0:
            self.position = max(self.position, -1*self.max_pos)

        # Now, publish the velocity and position to the truck
        if dist > 0.1:
            self.vel_pos_publisher(self.velocity, self.position)
            self.get_logger().info(f'Driving towards destination! Publishing[{self.i}] Velocity: {self.velocity}, Joint Position: {self.position}')
        # If destination arrived
        else:
            self.vel_pos_publisher(0.0, 0.0)
            self.get_logger().info(f'Destination Arrived Successfully!!')
        

def main(args=None):
    rclpy.init(args=args)

    closed_control = Autonomous()

    rclpy.spin(closed_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    closed_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
##-----------------------------------------------------------------------------##