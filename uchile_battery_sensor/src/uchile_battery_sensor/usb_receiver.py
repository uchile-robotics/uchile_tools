#!/usr/bin/python

__author__ = 'gdiaz'

import time
import bluetooth
import serial

class usbReceiver(object):
    def __init__(self, port, baud = 115200, debug = False):
        self.arduino = serial.Serial(port, baud, timeout=5)
        self.packet = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.debug = debug
        self.timeout = False

    def initialize(self):
        # TO DO: check port
        print "Conection succed! starting comunication ..."

    def stop(self):
        print "Conection Finish. Closing ports ..."
        self.arduino.close()

    def DEBUG_PRINT(self, msg_type, msg):
        if not(self.debug):
            return
        if msg_type == "info":
            print chr(27)+"[0;32m"+"[INFO]: "+chr(27)+"[0m" + msg
        elif msg_type == "warn":
            print chr(27)+"[0;33m"+"[WARN]: "+chr(27)+"[0m" + msg
        elif msg_type == "error":
            print chr(27)+"[0;31m"+"[ERROR]: "+chr(27)+"[0m" + msg
        elif msg_type == "alert":
            print chr(27)+"[0;34m"+"[ALERT]: "+chr(27)+"[0m" + msg
        else:
            print "NON implemented Debug print type"

    def reset(self):
        self.packet = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    def checksum(self, packet, sz):
        sum = 0
        for j in range(0,sz-1):
            sum += packet[j]
        return sum

    def write(self, frame):
        for i in xrange(0,len(frame)):
            self.arduino.write(chr(frame[i]))

    def read(self):
        i = 0
        k = 0
        sz = 14
        while (k < 2*sz):
            byte = self.arduino.read(1)
            self.packet[i] = ord(byte)
            i+=1
            if (i==sz):
                chksm = self.checksum(self.packet, sz) & 0x00FF #Low byte of data checksum
                if (chksm == self.packet[sz-1] and chksm !=0):
                    self.DEBUG_PRINT("info", "frame received = "+str(self.packet))
                    return True #packet received OK
                else:
                    for j in range(0,sz-1):
                        self.packet[j] = self.packet[j+1] #Shift Left packet
                    self.packet[sz-1] = 0 #Clean last byte to receive other packet
                    i = sz-1
                    self.DEBUG_PRINT("warn", "Bad checksum = "+str(chksm))
            k+=1
        # Packet not received Correctly
        self.DEBUG_PRINT("error", "Frame lost")
        self.reset()
        return False

if __name__ == '__main__':
    port = '/dev/ttyUSB0'
    baud = 9600
    usb_receiver = usbReceiver(port, baud, debug = True)
    usb_receiver.initialize()
    # test read
    while True:
        usb_receiver.read()
        print usb_receiver.packet
        usb_receiver.reset()
    # test write
    # Not implemented yet
    usb_receiver.stop()