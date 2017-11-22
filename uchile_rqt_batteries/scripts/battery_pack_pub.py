#!/usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
from uchile_rqt_batteries.battery_publisher import BatteryStatePublisher
from random import randint
from std_msgs.msg import Float32



class BatteryPackPublisher(BatteryStatePublisher):
	"""Publish notebook battery state"""
	def __init__(self, topic='battery_states', serial_number='battery', rate=1):
		super(BatteryPackPublisher, self).__init__(topic, serial_number, rate)

		self.soc_sub = rospy.Subscriber('/soc', Float32, self.get_battery_state)
		self.percentage=100.0
		self.charging=True

	def get_battery_state(self,percentage):
		# cmd = "upower -i $(upower -e | grep 'BAT') | grep -E 'state|to\ full|percentage'"
		# ps = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
		# output = ps.communicate()[0]
		# Get status
		#charging=False
		
		self.percentage=percentage.data
		# try:
		#  	percentage_pos=output.index('%')
		#  	percentage=float(output[percentage_pos-3:percentage_pos].strip())
		# except ValueError:
		#  	pass
		#return (charging, percentage)


	def spin(self):
		while not rospy.is_shutdown():
			#charging, percentage = self.get_battery_state()
			self.set_charging(self.charging)
			self.set_percentage(self.percentage)
			rospy.sleep(1.0)

def main():
	rospy.init_node('battery_pack_publisher', anonymous=True)
	# Get serial number
	serial_number = rospy.get_param('~serial_number', 'battery_pack')
	# Battery publisher
	bat_pub = BatteryPackPublisher(serial_number=serial_number)
	bat_pub.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: 
		pass
