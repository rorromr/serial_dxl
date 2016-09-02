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
    return result

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

def test_blink(dxl):
    for i in range(10):
        print dxl.write(1, 6, [0])
        print dxl.read(1, 0,8)[5:-2]
        time.sleep(0.05)
        dxl.write(1, 6, [1])
        print dxl.read(1, 0, 8)[5:-2]
        time.sleep(0.05)

def test_change_id(dxl):
    for i in range(5):
        print dxl.ping(1)
        #print "[0FF, 0FF, ID, LEN, ERR]"
        print dxl.read(1, 0, 7)[5:-2]
        print dxl.write(1,3,[2])
        time.sleep(0.5)
        #print dxl.read(1, 0, 8)[:13]
        print ''
        print dxl.read(2, 0, 7)[5:-2]
        print dxl.write(2,3,[1])
        time.sleep(0.5)

def test_ping(dxl):
    for i in range(5):
        print dxl.ping(1)

            
def main():
    dxl = DynamixelIO('/dev/ttyUSB1', baudrate = 1000000)
    #test_ping(dxl)
    test_blink(dxl)

        #time.sleep(0.5)

    #change_id(dxl, 5, 1)
    # blink(dxl,1)
    # while True:
    #     try:
    #         result = dxl.write(1, 6,[0x01, 0x01])
    #         result = dxl.read(1, 0, 8)
    #         print result
    #         result = dxl.write(1, 6,[0x00, 0x00])
    #         result = dxl.read(1, 0, 8)
    #         print result
    #         time.sleep(0.1)
    #     except:
    #         print "asdasd"
    #         time.sleep(1.0)
            
if __name__ == '__main__':
    main()
