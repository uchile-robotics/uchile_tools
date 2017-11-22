#!/usr/bin/python

__author__ = 'gdiaz'

# COM INTERFACE

from usb_receiver import usbReceiver
from file_manager import fileManager

class ComInterface:
    def __init__(self, test_name):
        # Only argument stuff
        self.running = False
        self.port = '/dev/ttyUSB0'
        self.baud = 9600
        self.usb_receiver = usbReceiver(self.port, self.baud, debug = True)
        self.file_manager = fileManager(test_name, debug = False)

    def initialize(self):
        self.running = True
        #USB interface
        self.usb_receiver.initialize()

    def stop(self):
        self.running = False
        self.usb_receiver.stop()
        self.file_manager.stop()

    def update_state(self):
        while True:
            # Receive data
            self.usb_receiver.read()
            packet = self.usb_receiver.packet
            self.usb_receiver.reset()
            #write data to file
            self.file_manager.save_data(packet)

if __name__ == '__main__':
    test_name = raw_input("test_name: ")
    comm = ComInterface(test_name)
    comm.initialize()
    comm.update_state()
    comm.stop()