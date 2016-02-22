#!/usr/bin/env python
import rospy
from mech_input.msg import LegMeasurementArray, LegMeasurement
# import csv


# f_handle = open('ped_data.csv','w')
# f_handle = open('ped_data_2.csv','w')
# f_handle = open('ped_data_5.csv','w')
# f_handle = open('ped_data_6.csv','w')
# f_handle = open('ped_data_7sim.csv','w')
# f_handle = open('ped_data_8sim.csv','w')
# f_handle = open('ped_data_1paper.csv','w')
# f_handle = open('ped_data_2paper.csv','w')
# f_handle = open('ped_data_3paper.csv','w')
f_handle = open('ped_data_4paper.csv','w')

def processLegArray(msg):
	print msg
	str_ = ''
	# if msg.no_of_legs == 1 and msg.Legs[0].xLeg == 0 and msg.Legs[0].yLeg == 0:
		# print msg
		# return
	if msg.no_of_legs > 0:
		for l in msg.Legs:
			# print l
			if l.xLeg == 0 and l.yLeg == 0:
				continue
			str_= str_  + format(l.xLeg,'.3f')+ ' '
			str_= str_  + format(l.yLeg,'.3f')+ ' '
			str_= str_  + format(l.ConLeg,'.3f')+ ' '
		str_ = str_ + '\n'
		print str_
		f_handle.write(str_)


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
		f_handle.close()
		pass


