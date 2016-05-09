#!/usr/bin/python

import sys
import time
from dynamixel_driver.dynamixel_io import DynamixelIO

def test_ping(dxl, dev_id):
    result = None
    try:
        result = dxl.ping(dev_id)
    except Exception as ex:
        print 'Exception thrown while pinging device %d - %s' % (dev_id, ex)
    
    if result:
        print 'Found %d' % (dev_id)
    return result

def test_read(dxl, dev_id, address):
    result = None
    try:
        result = dxl.read(dev_id, address,1)
    except Exception as ex:
        print 'Exception thrown while reading addres %d' % (address)

    if result:
        print 'Address %d = %d' % (address, result[5])
    return result[5]

def test_write(dxl, dev_id, address, data):
    result = None
    try:
        result = dxl.write(dev_id, address,[data])
    except Exception as ex:
        print 'Exception thrown while writing addres %d' % (address)

    if result:
        pass
        #print 'Response ' + str(result[5])
    return result

def blink(dxl, dev_id):
    test_write(dxl, 1, 6, 0x00)
    time.sleep(0.05)
    test_write(dxl, 1, 6, 0x01)
    time.sleep(0.05)

def change_id(dxl, dev_id, target_id):
    if test_ping(dxl, dev_id):
        test_write(dxl, dev_id, 3, target_id) # Use ID 5
        time.sleep(0.1)
        if test_ping(dxl, target_id):
            print 'Change ID Successful!'

            
def main():
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 115200)
    #change_id(dxl, 5, 1)
    while True:
        blink(dxl,1)

if __name__ == '__main__':
    main()
