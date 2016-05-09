#!/usr/bin/python

__author__ = 'Rodrigo Munoz'

import math
import rospy

from threading import Thread

from std_msgs.msg import Bool

# Use HW interface
from led_hw import LedHW, LED_ON, LED_OFF
from dynamixel_driver.dynamixel_io import DynamixelIO

class LedController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.led = LedHW(dxl_io, dev_id = 1)
        self.state = Bool()
        
    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        self.state.data = False

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Bool, self.process_command)
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', Bool, queue_size = 5)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.state_pub.unregister()

    def process_command(self, msg):
        com = 0
        if msg.data:
            com = 1
        self.led.command(com)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            current_state = self.led.get_state()
            if current_state == LED_ON:
                self.state.data = True
            else:
                self.state.data = False
            self.state_pub.publish(self.state)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('led_controller')
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
    led = LedController(dxl, 'led', 'left')
    led.initialize()
    led.start()
    rospy.spin()
    led.stop()
        
