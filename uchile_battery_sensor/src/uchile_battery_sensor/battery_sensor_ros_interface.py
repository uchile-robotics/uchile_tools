#!/usr/bin/python

__author__ = 'gdiaz'

# ROS INTERFACE

"""Provides a high level interface over ROS for current_sensor data over usb.
"""

import rospy

from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from usb_receiver import usbReceiver
from file_manager import fileManager

class USBRosInterface:
    def __init__(self, test_name):
        # Only argument stuff
        self.running = False
        self.port = '/dev/ttyUSB0'
        self.baud = 9600
        self.usb_receiver = usbReceiver(self.port, self.baud, debug = False)
        self.file_manager = fileManager(test_name, debug = False)
        self.power_sum = 0

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 1000)
        #USB interface
        self.usb_receiver.initialize()

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #publishers
        self.data1_pub = rospy.Publisher('/transfer_energy', Float32, queue_size=70)
        self.data2_pub = rospy.Publisher('/current', Float32, queue_size=70)
        self.data3_pub = rospy.Publisher('/soc', Float32, queue_size=70)
        self.data4_pub = rospy.Publisher('/voltage', Float32, queue_size=70)
        self.sum_power_pub = rospy.Publisher('/sum_power', Float32, queue_size=70)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.data1_pub.unregister()
        self.data2_pub.unregister()
        self.data3_pub.unregister()
        self.data4_pub.unregister()
        self.sum_power_pub.unregister()
        self.usb_receiver.stop()
        self.file_manager.stop()

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Receive data
            if(self.usb_receiver.read()):
                packet = self.usb_receiver.packet
                self.usb_receiver.reset()
                if packet[0] == 1:
                    #publish data
                    data = self.file_manager.decode(packet)
                    self.data1_pub.publish(data[0]/360)#temporal/360
                    self.data2_pub.publish(data[1])
                    self.data3_pub.publish(data[2])
                    self.data4_pub.publish(data[3])
                    self.power_sum=self.power_sum+data[0]*data[1]
                    self.sum_power_pub.publish(self.power_sum)
                    #write data to file
                    self.file_manager.save_data(packet, self.power_sum)
            rate.sleep()

if __name__ == '__main__':
    test_name = "battery_sensor_data_file"
    rospy.init_node('energy_meter_interface')
    usb_ros = USBRosInterface(test_name)
    usb_ros.initialize()
    usb_ros.start()
    rospy.spin()
    usb_ros.stop()