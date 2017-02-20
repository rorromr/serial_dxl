#!/usr/bin/python

import sys
import time
from dynamixel_driver.dynamixel_io import DynamixelIO
import struct
_struct = struct.Struct('<l')

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
        print dxl.read(1, 0, 7)
        time.sleep(0.05)
        print ''
        print dxl.write(1, 6, [1])
        print dxl.read(1, 0, 7)
        time.sleep(0.05)
        print ''

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

def serialize_int32(data):
    return  [ord(a) for a in list(struct.pack('<l',int(data)))]

          
def main():
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 200000)

    for i in range(100):
        try:
            dxl.get_feedback(21)
        except:
            print 'error'
    test_blink(dxl)
    for i in range(100):
        try:
            dxl.get_feedback(21)
        except:
            print 'error'
    test_blink(dxl)

    #test_write(dxl, 1, 4, 9)

    #test_ping(dxl)
    #test_blink(dxl)
    # target = -900.63
    # data = serialize_int32(target*1000)
    # print (data[0])+(data[1]<<8)+(data[2]<<16)+(data[3]<<24)
    # print data
    # print dxl.write(1,7,data)


   

    # print [b1,b2,b3,b4]

    # print (b1)+(b2<<8)+(b3<<16)+(b4<<24)
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
