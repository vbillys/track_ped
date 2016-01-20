#!/usr/bin/env python
import rospy
from mech_input.msg import LegMeasurementArray, LegMeasurement


def processLegArray(msg):
	print msg
	# for l in msg.Legs:
		# print l

def talker():
	rospy.init_node('track_ped', anonymous=False)
	# rospy.Subscriber('/usb_cam/image_raw', Image, filter_points)
	rospy.Subscriber('/legs', LegMeasurementArray, processLegArray)
	rospy.spin()
	pass

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

