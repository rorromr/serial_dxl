#!/usr/bin/python

import time
from dynamixel_driver.dynamixel_io import DynamixelIO

# NON ROS HARDWARE INTERFACE

LED_STATE = 6
LED_OFF = 0
LED_ON = 1

class LedHW(object):
    def __init__(self, dxl_io, dev_id = 1):
        self.dxl = dxl_io
        self.id = dev_id
        self.state = 0

    def ping(self):
        result = []
        try:
            result = self.dxl.ping(self.id)
        except Exception as e:
            print 'Exception thrown while pinging device %d - %s' % (self.id, e)
            raise e
        return result

    def get_state(self):
        result = []
        try:
            result = self.dxl.read(self.id, LED_STATE, 1)
        except Exception as e:
            print 'Exception thrown while reading addres %d' % (LED_STATE)
            return e
        self.state = result[5]
        return self.state

    def command(self, com = 1):
        if (com != LED_ON) and (com != LED_OFF):
            print 'Unknown command %d' % (com)
        
        result = []
        try:
            result = self.dxl.write(self.id, LED_STATE, [com])
        except Exception as e:
            print 'Exception thrown while writing addres %d' % (LED_STATE)
            raise e
        return result

    def turn_on(self):
        return self.command(LED_ON)

    def turn_off(self):
        return self.command(LED_OFF)

if __name__ == '__main__':
    import time

    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
    led = LedHW(dxl, dev_id = 1)
    while True:
        led.turn_on()
        time.sleep(0.5)
        led.turn_off()
        time.sleep(0.5)

