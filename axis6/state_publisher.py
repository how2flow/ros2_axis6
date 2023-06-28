# Copyright 2023 how2flow
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        # vars
        self.ch = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.degree = 0.01
        self.joint_state = JointState()
        self.loop_rate = self.create_rate(30)

        # qos
        qos_profile = QoSProfile(depth=10)

        # publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

def main():
    rclpy.init()
    try:
        node = StatePublisher()
        try:
            while rclpy.ok():
                rclpy.spin_once(node)

                now = node.get_clock().now()
                node.joint_state.header.stamp = now.to_msg()
                node.joint_state.name = ['ch0', 'ch1', 'ch2', 'ch3', 'ch4']
                node.joint_state.position = [node.ch[0], node.ch[1], node.ch[2], node.ch[3], node.ch[4]]

                node.ch[0] += node.degree
                if node.ch[0] > pi:
                    node.degree = -0.01
                if node.ch[0] < -pi:
                    node.degree = 0.01

                node.joint_pub.publish(node.joint_state)
                node.loop_rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
