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

import rclpy
from rclpy.node import Node
from how2flow_interfaces.srv import PairInt8

class Axis6Commander(Node):

    def __init__(self):
        super().__init__('axis6_commander')
        # service-client
        self.axis6_service_client = self.create_client(
            PairInt8,
            'axis_command')

        # wait service
        while not self.axis6_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The operaor service not available')

    def send_request(self):
        service_request = PairInt8.Request()
        service_request.p1, service_request.p2 = \
            map(int, input('ch pos: ').split())
        futures = self.axis6_service_client.call_async(service_request)
        return futures

def main(args=None):
    rclpy.init(args=args)
    node = Axis6Commander()
    future = node.send_request()
    trigger = True
    try:
        while rclpy.ok():
            if trigger is True:
                rclpy.spin_once(node)
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        node.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                        node.get_logger().info(
                            'Result: {}'.format(service_response.state))
                        trigger = False
            else:
                # input('Press Enter for next service call')
                future = node.send_request()
                trigger = True
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
