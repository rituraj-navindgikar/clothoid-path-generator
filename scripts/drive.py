import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

import numpy as np

class DriveVehicle(Node):
    def __init__(self, *args, **kwargs):
        super().__init__('driving_node')
        self.steer_publisher = self.create_publisher(JointTrajectory, '/steering/steering_trajectory', 10)
        self.timer = self.create_timer(0.1, self.publish_steer_angle)

        self.linear_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def publish_steer_angle(self, angular_vel):
        msg = JointTrajectory()
        msg.header.frame_id = 'world'
        msg.joint_names = ['top_left_steer_joint', 'top_right_steer_joint']
        point = JointTrajectoryPoint()
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 1

        angular_vel = np.clip(angular_vel, -0.4, 0.4)
        point.positions = [angular_vel, angular_vel]

        msg.points.append(point)

        self.steer_publisher.publish(msg)

    
    def publish_cmd_vel(self, linear_vel):
        msg = Twist()
        
        linear_vel = np.clip(linear_vel, -2, 2)
        msg.linear.x = linear_vel
        
        
        self.linear_vel_publisher.publish(msg)


def main():
    rclpy.init()
    node_drive = DriveVehicle()

    rclpy.spin(node_drive)
    node_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
