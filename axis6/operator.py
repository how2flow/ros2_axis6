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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from how2flow_interfaces.msg import Dict
from how2flow_interfaces.srv import PairInt8

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

import odroid_wiringpi as wpi
import time

# adc
ADC_MAX = 4095 # 4Kb - 1
ADC_ERR = 204.75 # 5%

# servo
BASIC_POS = 6
MAX_RESOLUTION = 0xFFFF

# pin numbers (wpi)
DIP_PINS = [14, 13, 12, 3, 2, 0]
SW = 4
JOY_X = 29
JOY_Y = 25

# flag
ON = 1
OFF = 0

# direction
POSITIVE = 1
NEGATIVE = -1

# mode
NORMAL = 0
DISPLAY = 1
SERVICE = 2

class Axis6Operator(Node):

    def __init__(self):
        super().__init__('axis6_operator')

        # vars
        self.ch = 0
        self.dip = 0
        self.motor = \
            {0: BASIC_POS, 1: BASIC_POS, 2: BASIC_POS, 3: BASIC_POS, 4: BASIC_POS, 5: BASIC_POS}
        self.dir = [POSITIVE, NEGATIVE, POSITIVE, NEGATIVE, NEGATIVE, POSITIVE] # POSITIVE or NEGATIVE
        self.res = 0
        self.mode = NORMAL
        self.neutral = OFF
        self.step = 0.2618 # 15 x (pi / 180)

        # odroid_wiringpi
        wpi.wiringPiSetup()

        for pin in DIP_PINS:
            wpi.pinMode(pin, 0)

        wpi.pinMode(SW, 0)

        # pca9685
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c_bus)
        self.pca.frequency = 50

        # homing
        for i in range(len(DIP_PINS)):
            self.pca.channels[i].duty_cycle = int((MAX_RESOLUTION * BASIC_POS) / 100)

        # callback
        self.callback_group = ReentrantCallbackGroup()

        # qos
        qos_profile = QoSProfile(depth=10)

        # publisher - create
        self.state_publisher = self.create_publisher(
            Dict,
            'axis_state',
            qos_profile)

        # publisher - timer
        self.create_timer(0.4, self.publish_axis_state)

        # subscription
        self.subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.subscribe_joint_state,
            qos_profile)

        # service
        self.axis6_service_server = self.create_service(
            PairInt8,
            'axis_command',
            self.service_axis6,
            callback_group=self.callback_group)

    def select_mode_and_channel(self):
        self.dip = 0
        self.neutral = OFF
        channels = 0x3f # 0b111111
        mask = 0x20 # 0b100000

        for pin in DIP_PINS:
            self.dip <<= 1
            self.dip += (wpi.digitalRead(pin))

        for ch in range(len(DIP_PINS)):
            if (self.dip | mask == channels):
                self.ch = ch
                break
            mask >>= 1

        if (self.dip == channels):
            self.mode = NORMAL
            self.neutral = ON
            self.ch = 0

        if (self.dip == 0):
            self.mode = DISPLAY
            self.neutral = OFF

    def motor_write(self, ch, x, y):
        orig_pos = self.motor[ch]

        # VRx
        if (abs(y - (ADC_MAX / 2)) < ADC_ERR):
            if ((ch % 4) == 0):
                if (x < ADC_ERR): # left
                    self.motor[ch] += self.dir[ch]

                elif (abs(ADC_MAX - x) < ADC_ERR): # right
                    self.motor[ch] -= self.dir[ch]

        # VRy
        if (abs(x - (ADC_MAX / 2)) < ADC_ERR):
            if ((ch % 4) > 0):
                if (y < ADC_ERR): # up
                    self.motor[ch] += self.dir[ch]
                elif (abs(ADC_MAX - y) < ADC_ERR): # down
                    self.motor[ch] -= self.dir[ch]

        if (abs(BASIC_POS - self.motor[ch]) > 1):
            self.motor[ch] = orig_pos

    def motor_control(self, ch, pos):
        if (ch < -1 or ch > 5):
            self.get_logger().warning('ch: The input value is out of range.')
            self.res = -1
            return self.res

        if (ch == 5):
            if (abs(pos - BASIC_POS) > 2):
                self.get_logger().warning('ch: The input value is out of range.')
                self.res = -1
                return self.res

        elif (ch == -1):
            self.get_logger().warning('ch: neutral')
            self.res = -1
            return self.res

        else:
            if (abs(pos - BASIC_POS) > 4):
                self.get_logger().warning('ch: The input value is out of range.')
                self.res = -1
                return self.res

        if (self.neutral == OFF):
            self.pca.channels[ch].duty_cycle = int((MAX_RESOLUTION * pos) / 100)

        self.res = 1

        return self.res

    def publish_axis_state(self):
        msg = Dict()

        if (self.mode != SERVICE or wpi.digitalRead(SW) == 0):
            self.select_mode_and_channel()

        if (self.mode == NORMAL):
            self.motor_write(self.ch, wpi.analogRead(JOY_X), wpi.analogRead(JOY_Y))
            self.motor_control(self.ch, self.motor[self.ch])
            msg.idx = self.ch
            msg.data = float(self.motor[self.ch]) # temporary

            if (self.neutral == OFF):
                self.state_publisher.publish(msg)

    def subscribe_joint_state(self, msg):
        if (self.mode == DISPLAY):
            for i in range(len(DIP_PINS) - 1): # except gripper
                self.motor[i] = self.convert_step(i, msg.position[i])
                if (abs(BASIC_POS - self.motor[i]) < 3):
                    self.motor_control(i, self.motor[i])

    def convert_step(self, ch, rad):
        offset = self.dir[ch] * int(rad / self.step)
        return BASIC_POS + offset

    def service_axis6(self, request, response):
        self.mode = SERVICE
        self.neutral = OFF

        response.state = self.motor_control(request.p1, request.p2)

        msg = Dict()
        msg.idx = request.p1
        msg.data = float(request.p2)
        self.state_publisher.publish(msg)

        self.get_logger().info('<service control> ch: {0} pos: {1}'.format(
            request.p1,
            request.p2))

        return response

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Axis6Operator()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt')
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
