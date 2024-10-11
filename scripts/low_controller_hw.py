#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from pydrake.all import (
    BasicVector,
    Context,
    LeafSystem,
    Value,
)

from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from leap_ros2.utils import convert_ros_traj_to_drake_traj, time_msg_to_float


class LowlevelControllerHw(Node):
    def __init__(self):
        super().__init__('low_level_controller_node')  # 节点名称

        low_level_ctrl_freq = 100
        low_level_dt = 1 / low_level_ctrl_freq
        
        # # 创建一个发布者
        # self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        
        self.high_level_traj_subscriber_ = self.create_subscription(
            JointTrajectory,
            'high_level_traj',
            self.high_level_traj_callback,
            100
        )

        self.hw_command_publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10
        )
        
        # 设置定时器，每隔1秒发布消息
        self.timer = self.create_timer(low_level_dt, self.timer_callback)

        _client = self.create_client(GetParameters, '/joint_position_controller/get_parameters')
        if not _client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Parameter service not available')
            return
        
        # request = GetParameters.Request()
        # request.names = ['joints']  # 查询 'joints' 参数
        # future = _client.call_async(request)
        self.hw_joint_names = [f"joint_{id}" for id in range(16)]
        self.get_logger().info('Hardware takes joints in the following order: %s' % self.hw_joint_names)

        self.high_level_traj = None
        self.joint_names = None
        self.num_joints = 0
        self.remapping_to_hw = []

    def high_level_traj_callback(self, msg:JointTrajectory):
        high_level_traj = convert_ros_traj_to_drake_traj(msg)
        self.high_level_traj = high_level_traj
        if self.num_joints == 0:
            self.joint_names = msg.joint_names
            self.num_joints = len(msg.joint_names)
            for name in self.hw_joint_names:
                self.remapping_to_hw.append(self.joint_names.index(name))
            assert len(self.remapping_to_hw) == self.num_joints

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        current_time = time_msg_to_float(current_time)

        joint_commands = None
        if self.high_level_traj is None:
            # joint_commands = np.zeros(self.num_joints)
            return
        else:
            joint_commands = self.high_level_traj.value(current_time).flatten()
            joint_commands = joint_commands[self.remapping_to_hw]

        command_msg = Float64MultiArray()
        command_msg.data = joint_commands.tolist()
        self.hw_command_publisher_.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    low_level_ctrl_node = LowlevelControllerHw()

    try:
        rclpy.spin(low_level_ctrl_node)
    except KeyboardInterrupt:
        pass
    finally:
        low_level_ctrl_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
