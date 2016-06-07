#!/usr/bin/env python
import rospy
from arcleg_segmentation.msg import LegMeasurementArray, LegMeasurement , PersonTrack, PersonTrackArray

from munkres import Munkres
from scipy.spatial.distance import mahalanobis
from scipy.stats import multivariate_normal
import math
import time
import numpy as np
import matplotlib
#matplotlib.use('GTKAgg')
import pylab as plt
# import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import tf

from CustomCreateKF import createLegKF, createPersonKF, squareMatrix

print 'start track ros'

g_pub_ppl = None
g_pub_marker = None
#<<<<<<< Updated upstream
#g_use_display = False #True #False
#g_use_decay_when_nodata =False # True #False #True
g_use_raw_leg_data = False
g_no_ppl_predict_when_update_fail = False #True
# <<<<<<< Updated upstream
g_use_limit_ppl_predict = True
# g_use_limit_ppl_predict = False
# =======
# >>>>>>> Stashed changes
#=======
g_use_display = False #True #True #True #False
g_use_decay_when_nodata = True #False #True
g_use_raw_leg_data = False
g_use_clip = False
#>>>>>>> Stashed changes

g_use_odom = True
g_odom_topic = 'RosAria/pose'

COST_MAX_GATING = .8 #1.5 #.7 #1.5 #.7 #1.5
COST_MAX_GATING_ONELEG = .8 #1.5 #.7 #1.5 #.7 #1.5
DECAY_RATE = 0.95 #0.93
DECAY_THRES = 0.3
DECAY_RATE_LEG = .90 #0.8 #93
DECAY_THRES_LEG = .5 #0.3
IMPROVE_RATE = 0.05 #0.02 #0.05
PERSON_CONFIRM_THRES = 0.5
RMAHALANOBIS = 2. #.5 #2.5 #2.5
MAX_DIST_PERSON_ONELEG = 1 #.5 #1 #.3
PERSON_GATING_DISTANCE = 0.8
MAX_DIST_OWNERSHIP_ONELEG = .5 #.35#.5
LIMIT_PPL_PREDICT = .03 #0.015 #.03

PERSON_KF_pd = .5 #25.
PERSON_KF_pv = .2 #10.
PERSON_KF_pa = .5 #30.
PERSON_KF_rd = 0.01
PERSON_KF_rv = 0.1
PERSON_KF_ra = 1.
PERSON_KF_q  = 0.7
LEG_KF_pd = .5
LEG_KF_pv = .2
LEG_KF_rd = 0.05
LEG_KF_rv = 0.2
LEG_KF_q  = 0.7

def getVMatrixCAModel(k_filter):
	V = np.array([[k_filter.P[0,0],k_filter.P[0,3]],[k_filter.P[3,0],k_filter.P[3,3]]])
	V = V + np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]])
	return V

def getPosPMatrixCAModel(k_filter):
	# return np.array([[k_filter.P[0,0],k_filter.P[0,3]],[k_filter.P[3,0],k_filter.P[3,3]]]) + np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]])
	return np.array([[k_filter.P[0,0],k_filter.P[0,3]],[k_filter.P[3,0],k_filter.P[3,3]]])

def calcMahalanobisDistance(first_p, second_p, V):
	return mahalanobis (np.array([first_p[0], first_p[1]]), np.array([second_p[0], second_p[1]]), np.linalg.inv(V))

def isThereAnyOneLegAround(KF_point,k_filter, onelegs_track, onelegs_track_owned_index, onelegs_track_owning_person_index, person_index):

	index = 0
	indexes = []
	mdists = []
	edists = []
	ratio_dists = []
	weights = []
	no_decay = False
	# var = multivariate_normal(mean=[KF_point[0],KF_point[1]], cov=getPosPMatrixCAModel(k_filter))
	var = multivariate_normal(mean=[KF_point[0],KF_point[1]], cov=np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]]))
	var0= var.pdf([KF_point[0],KF_point[1]])
	# print k_filter
	V = getVMatrixCAModel(k_filter)

	for oneleg in onelegs_track:
		_mdist = mahalanobis(np.array([oneleg[1], oneleg[2]]),
				np.array([KF_point[0],KF_point[1]]),
				np.linalg.inv(V))
		_edist = math.hypot(oneleg[1] - KF_point[0], oneleg[2] - KF_point[1])
		# if _mdist < COST_MAX_GATING_ONELEG:
		if _edist < MAX_DIST_PERSON_ONELEG:
			no_decay = True
			index_is_the_person = False
			if index in onelegs_track_owned_index:
				if person_index == onelegs_track_owning_person_index[onelegs_track_owned_index.index(index)]:
					index_is_the_person = True

			if index_is_the_person or index not in onelegs_track_owned_index:
				indexes.append(index)
				mdists.append(_mdist)
				edists.append(_edist)
				ratio_dists.append(_mdist/_edist)
				weights.append(var.pdf([oneleg[1], oneleg[2]]))
		index = index + 1
	# indexes = []
	return  indexes, mdists, edists, ratio_dists, sum(ratio_dists), weights, sum(weights)+var0, var0, no_decay

def findMinIndexFromOnelegsTrack(_xx , _yy, onelegs_track, thres):
	dist = []
	# mindist = []
	oneleg_index = []
	i = 0
	for oneleg in onelegs_track:
		_dist = math.hypot(_xx - oneleg[1], _yy - oneleg[2])
		if _dist < thres:
			dist.append(_dist)
			# mindist.append(mindist)
			oneleg_index.append(i)
		i = i + 1
	if dist:
		return oneleg_index[dist.index(min(dist))]
	else:
		return None

# mx, my = getMMSEOneLegs(_check, onelegs_track)
def getMMSEOneLegs(check, onelegs_track, R, KF_point, P):
	_noidx = 0
	mmse_x = 0
	mmse_y = 0
	P_mmse = np.array([[0,0],[0,0]])
	# print 'check:', check
	for index in check[0]:
		# _ratio = check[3][_noidx]/check[4]
		_ratio = check[5][_noidx]/check[6]
		mmse_x = mmse_x + (onelegs_track[index][1]-KF_point[0])*_ratio
		mmse_y = mmse_y + (onelegs_track[index][2]-KF_point[1])*_ratio
		_noidx = _noidx + 1
	mmse_x = mmse_x + KF_point[0]
	mmse_y = mmse_y + KF_point[1]
	_noidx = 0
	for index in check[0]:
		# _ratio = check[3][_noidx]/check[4]
		_ratio = check[5][_noidx]/check[6]
		_delta_x = onelegs_track[index][1] - mmse_x
		_delta_y = onelegs_track[index][2] - mmse_y
		P_mmse = P_mmse + (R + np.array([_delta_x,_delta_y])*np.array([[_delta_x],[_delta_y]]))*_ratio
		_noidx = _noidx + 1
	# P_mmse = P_mmse + P*check[7] #+ np.array([[3,0],[0,3]])
	P_mmse = P_mmse + R*check[7] #+ np.array([[3,0],[0,3]])
	# print 'P-mmse:', P_mmse


	return mmse_x , mmse_y, P_mmse


def rotateYaw( coord_2d, yaw):
	coord_2d_new_x = math.cos(yaw)*coord_2d[0] - math.sin(yaw)*coord_2d[1]
	coord_2d_new_y = math.sin(yaw)*coord_2d[0] + math.cos(yaw)*coord_2d[1]
	return [coord_2d_new_x, coord_2d_new_y]


class PeopleTrackerFromLegs:
	def __init__(self, display, pub_persons, pub_marker, use_display, use_decay_when_nodata, use_raw_leg_data, no_ppl_predict_when_update_fail,use_limit_ppl_predict , use_odom = False, odom_topic = None):
		self.use_display = use_display
		self.use_decay_when_nodata = use_decay_when_nodata
		self.no_ppl_predict_when_update_fail = no_ppl_predict_when_update_fail
		self.use_limit_ppl_predict = use_limit_ppl_predict
		self.use_raw_leg_data = use_raw_leg_data
		self.munkres = Munkres()
		self.kalman_filters_leg = []
		self._first_leg = False
		# self.tracks_leg = []
		# self.tracks_KF_leg = []
		# self.tracks_conf_leg = []
		# self.tracks_KF_points_leg = []
		# self.track_leg = []
		self.track_KF_leg = []
		self.track_conf_leg = []
		self.track_KF_point_leg = []
		self.track_size_leg = []
		# _frame_idx = 0
		self._obj_id = 1
		# self.max_obj_id = rospy.get_param('~max_id', 64)
		self.max_obj_id = MAX_OBJ_ID 
		self.display = display

		self._first_people = False
		self.kalman_filters_people = []
		self.track_KF_people = []
		self.track_conf_people = []
		self.track_KF_point_people = []
		self._people_id = 1
		self.track_KF_confirmations = []
		self.track_KF_improvements = []
		self.track_KF_onelegmode = []

		self.pub_persons = pub_persons
		self.pub_marker = pub_marker

		# KF Constants
		# self.SAMPLING_RATE = rospy.get_param('~sampling_rate', 40)
		self.SAMPLING_RATE = SAMPLING_RATE
		self.KF_DT = 1/self.SAMPLING_RATE
		self.KF_DT2 = self.KF_DT*self.KF_DT/2
		self.odom_topic = odom_topic
		self.use_odom = use_odom

		self.init_odom = False
		self.first_odom_calculated = False
		self.x_pose_last = 0
		self.y_pose_last = 0
		self.heading_pose_last = 0
		self.x_pose= 0
		self.y_pose= 0
		self.heading_rad = 0
		self.x_diff = 0
		self.y_diff = 0
		self.heading_rad_diff = 0
		#if self.use_odom:
		rospy.Subscriber(self.odom_topic, Odometry, self.processOdomMsg)

	def processOdomMsg(self, msg):
		if self.init_odom:
			quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			self.heading_rad = euler[2] - self.first_odom_euler[2]
			self.x_pose = msg.pose.pose.position.x - self.first_odom_msg.pose.pose.position.x
			self.y_pose = msg.pose.pose.position.y - self.first_odom_msg.pose.pose.position.y

			# print 'x: %.3f y: %.3f @: %.3f\nDiffs:\nx: %.3f y: %.3f @: %.3f' % (self.x_pose, self.y_pose, self.heading_rad, self.x_diff, self.y_diff, self.heading_rad_diff)
			self.first_odom_calculated = True
			self.shiftTrackIfYawRateAboveThreshold( msg.twist.twist.angular.z)
		else:
			self.first_odom_msg = msg
			self.first_odom_quat = (self.first_odom_msg.pose.pose.orientation.x, self.first_odom_msg.pose.pose.orientation.y, self.first_odom_msg.pose.pose.orientation.z, self.first_odom_msg.pose.pose.orientation.w)
			self.first_odom_euler = tf.transformations.euler_from_quaternion(self.first_odom_quat)
			#self.calculateOdomDiffAndUpdate()
		self.init_odom = True

	def calculateOdomDiffAndUpdate(self):
		if not self.first_odom_calculated:
			return
		if not self.use_odom:
			return
		# self.x_diff = msg.pose.pose.position.x - self.last_odom_msg.pose.pose.position.x
		# self.y_diff = msg.pose.pose.position.y - self.last_odom_msg.pose.pose.position.y
		self.x_diff = self.x_pose - self.x_pose_last
		self.y_diff = self.y_pose - self.y_pose_last
		# last_quat = (self.last_odom_msg.pose.pose.orientation.x, self.last_odom_msg.pose.pose.orientation.y, self.last_odom_msg.pose.pose.orientation.z, self.last_odom_msg.pose.pose.orientation.w)
		# last_euler = tf.transformations.euler_from_quaternion(last_quat)
		# self.heading_rad_diff = euler[2] - last_euler[2]
		self.heading_rad_diff = self.heading_rad - self.heading_pose_last
		# self.last_odom_msg = msg
		self.x_pose_last = self.x_pose
		self.y_pose_last = self.y_pose
		self.heading_pose_last = self.heading_rad
		# print "pose diffs: %.3f, %.3f, %.3f" % (self.x_diff, self.y_diff, self.heading_rad_diff)

	def shiftTrackIfYawRateAboveThreshold(self, twistz):
		# if twistz > math.radians(.3*10): # sampling rate of odometry
		self.rotateTrackWithYaw(-twistz/10)

	def generateNewLegId(self):
		self._obj_id = self._obj_id + 1
		if self._obj_id > self.max_obj_id: 
			self._obj_id = 1

	def generateNewPersonId(self):
		self._people_id = self._people_id+ 1
		if self._people_id> self.max_obj_id: 
			self._people_id= 1


	def odomTranslateAndThenRotate(self, coord_2d):
		coord_2d[0] = coord_2d[0] - self.x_pose
		coord_2d[1] = coord_2d[1] - self.y_pose
		coord_2d_new_x = math.cos(-self.heading_rad)*coord_2d[0] - math.sin(-self.heading_rad)*coord_2d[1]
		coord_2d_new_y = math.sin(-self.heading_rad)*coord_2d[0] + math.cos(-self.heading_rad)*coord_2d[1]
		return [coord_2d_new_x, coord_2d_new_y]

	def odomTranslateAndThenRotate2(self, coord_2d):
		coord_2d_new_x = math.cos(self.heading_rad)*coord_2d[0] - math.sin(self.heading_rad)*coord_2d[1]
		coord_2d_new_y = math.sin(self.heading_rad)*coord_2d[0] + math.cos(self.heading_rad)*coord_2d[1]
		coord_2d_new_x  = coord_2d_new_x + self.x_pose
		coord_2d_new_y  = coord_2d_new_y + self.y_pose
		return [coord_2d_new_x, coord_2d_new_y]

	def shiftMeasurementAccordingOdomDiff(self, points):
		points_new = []
		for leg in points:
			leg_new = leg
			_xd = [leg[0], leg[1]]
			_xd = self.odomTranslateAndThenRotate2(_xd)
			leg_new[0] = _xd[0]
			leg_new[1] = _xd[1]
			points_new.append(leg_new)
		return points_new

	def rotateTrackWithYaw(self, yaw):
		for kf in self.kalman_filters_leg:
			_x = kf.x
			_xd = [_x[0], _x[2]]
			_xd = rotateYaw(_xd, yaw)
			_xv = [_x[1], _x[3]]
			_xv = rotateYaw(_xv, yaw)
			kf.x = [_xd[0], _xv[0], _xd[1], _xv[1]]
		for kf in self.kalman_filters_people:
			_x = kf.x
			_xd = [_x[0], _x[3]]
			_xd = rotateYaw(_xd, yaw)
			_xv = [_x[1], _x[4]]
			_xv = rotateYaw(_xv, yaw)
			_xa = [_x[2], _x[5]]
			_xa = rotateYaw(_xa, yaw)
			kf.x = [_xd[0], _xv[0], _xa[0], _xd[1], _xv[1], _xa[1]]

	def shiftTrackAccordingOdomDiff(self):
		for kf in self.kalman_filters_leg:
			_x = kf.x
			_xd = [_x[0], _x[2]]
			_xd = self.odomTranslateAndThenRotate(_xd)
			_xv = [_x[1], _x[3]]
			_xv = self.odomTranslateAndThenRotate(_xv)
			kf.x = [_xd[0], _xv[0], _xd[1], _xv[1]]
		for kf in self.kalman_filters_people:
			_x = kf.x
			_xd = [_x[0], _x[3]]
			_xd = self.odomTranslateAndThenRotate(_xd)
			_xv = [_x[1], _x[4]]
			_xv = self.odomTranslateAndThenRotate(_xv)
			_xa = [_x[2], _x[5]]
			_xa = self.odomTranslateAndThenRotate(_xa)
			kf.x = [_xd[0], _xv[0], _xa[0], _xd[1], _xv[1], _xa[1]]

	def processMunkresKalman(self,points):

		# self.calculateOdomDiffAndUpdate()
		# self.shiftTrackAccordingOdomDiff(points)
		if self.use_odom:
			points = self.shiftMeasurementAccordingOdomDiff(points)
		# for frame in points:
		frame = points
		self.points = points
		# munkres = Munkres()
		# print frame
		if len(frame)>0:
			if not self._first_leg:
				self._first_leg = True
				# track = []
				# track_conf = []
				# track_KF_point = []
				for leg in frame:
					self.track_KF_leg.append(self._obj_id)
					self.kalman_filters_leg.append(createLegKF(leg[0], leg[1], self.KF_DT, KF_pd = LEG_KF_pd, KF_pv = LEG_KF_pv, KF_rd = LEG_KF_rd, KF_rv = LEG_KF_rv, KF_q = LEG_KF_q))
					self.track_conf_leg.append(leg[2])
					self.track_size_leg.append(leg[3])
					# self.track_KF_point_leg.append([leg[0],leg[1]])
					self.track_KF_point_leg.append([leg[0],leg[1],0 ,0])
					self.generateNewLegId()
				# self.display.setup_plot(frame, self.track_KF_point_leg)
				# tracks.append(track)
				# tracks_KF.append(track)
				# tracks_conf.append(track_conf)
				# tracks_KF_points.append(track_KF_point)
				# print track
				# track_KF = track
				# last_frame_idx = _frame_idx
				# print self.track_KF_leg, self.kalman_filters_leg
			else:
				track_KF_point_new = []
				for kf in self.kalman_filters_leg:
					kf.predict()
					_x_updated = kf.x
					# track_KF_point_new.append([_x_updated[0], _x_updated[2]])
					track_KF_point_new.append([_x_updated[0], _x_updated[2],_x_updated[1], _x_updated[3]])
				self.track_KF_point_leg = track_KF_point_new
				cost_matrix = []
				# for prev_leg in points[last_frame_idx]:
				_lidx = 0
				# no_of_object = 0
				# valid_ids = []
				# valid_lidxs = []
				for objid in self.track_KF_leg:
					# if objid != 0:
						# valid_ids.append(objid)
						# valid_lidxs.append(_lidx)
					cost_matrix.append([])
					# print _lidx, objid, self.track_KF_leg, self.kalman_filters_leg
					# no_of_object = no_of_object + 1
					V = np.array([[self.kalman_filters_leg[_lidx].P[0,0],self.kalman_filters_leg[_lidx].P[0,2]],[self.kalman_filters_leg[_lidx].P[2,0],self.kalman_filters_leg[_lidx].P[2,2]]])
					# V = np.array([[kalman_filters[_lidx].S[0,0],kalman_filters[_lidx].S[0,2]],[kalman_filters[_lidx].S[2,0],kalman_filters[_lidx].S[2,2]]])
					# V = kalman_filters[_lidx].S

					# V = dot3(kalman_filters[_lidx].H, kalman_filters[_lidx].P, kalman_filters[_lidx].H.T) + np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]])
					V = V + np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]])
					# print V
					for leg in frame:
						# _dist = math.hypot(prev_leg[0] - leg[0], prev_leg[1] - leg[1])
						# _mdist = mahalanobis(np.array([points[last_frame_idx][_lidx][0], points[last_frame_idx][_lidx][1]]),
							    # np.array([leg[0],leg[1]]),
							    # np.linalg.inv(V))
						_mdist = mahalanobis(np.array([self.track_KF_point_leg[_lidx][0], self.track_KF_point_leg[_lidx][1]]),
								np.array([leg[0],leg[1]]),
								np.linalg.inv(V))
						# print V, _mdist
						# _dist = math.hypot(points[last_frame_idx][_lidx][0] - leg[0], points[last_frame_idx][_lidx][1] - leg[1])

						# _dist = math.hypot(track_KF_point[_lidx][0] - leg[0], track_KF_point[_lidx][1] - leg[1])
						# cost_matrix[-1].append(_dist)
						cost_matrix[-1].append(_mdist)
					_lidx = _lidx + 1
				# print _frame_idx, cost_matrix

				total = 0.
				# track_new = []
				track_KF_new = []
				track_conf_new = []
				track_size_new = []
				track_KF_point_new = []
				kalman_filters_new = []
				rows = []
				columns = []
				indexes = []

				if _lidx > 0:
					cost_matrix = squareMatrix(cost_matrix, COST_MAX_GATING)
					# print cost_matrix
					indexes = self.munkres.compute(cost_matrix)

				for row, column in indexes:
					# track_new.append(valid_ids[row])
					value = cost_matrix[row][column]
					total += value
					# print '(%d, %d) -> %f' % (row, column, value)
				# print 'total cost: %f' % total
				for row, column in indexes:
					rows.append(row)
					columns.append(column)
					# track_new.append(track[row])
				# for i in range(len(points[last_frame_idx])):
				# print rows, columns

				for i in range(len(frame)):
					# if i not in columns: # or cost_matrix[rows[columns.index(i)]][i] > COST_MAX_GATING:
					if i not in columns or cost_matrix[rows[columns.index(i)]][i] >= COST_MAX_GATING:
						# add new obj id for unassigned measurements
						# print 'added new object'
						# track_new.append(_obj_id)
						kalman_filters_new.append(createLegKF(frame[i][0], frame[i][1], self.KF_DT, KF_pd = LEG_KF_pd, KF_pv = LEG_KF_pv, KF_rd = LEG_KF_rd, KF_rv = LEG_KF_rv, KF_q = LEG_KF_q))
						track_KF_point_new.append([frame[i][0], frame[i][1],0,0])
						track_KF_new.append(self._obj_id)
						track_conf_new.append(frame[i][2])
						track_size_new.append(frame[i][3])
						self.generateNewLegId()
					else:
						# print i, columns.index(i), rows[columns.index(i)], cost_matrix
						# if cost_matrix[rows[columns.index(i)]][i] >= COST_MAX_GATING:
							# continue
						# unassigned tracked ids (Munkres) die immediately
						# compute for assigned tracks
						# track_new.append(self.track_KF_leg[rows[columns.index(i)]])
						kalman_filters_new.append(self.kalman_filters_leg[rows[columns.index(i)]])
						# kalman_filters[rows[columns.index(i)]].update(np.array([frame[i][0], frame[i][1]]))

						# kalman_filters[rows[columns.index(i)]].update([frame[i][0], frame[i][1]])
						self.kalman_filters_leg[rows[columns.index(i)]].update([frame[i][0], frame[i][1], (frame[i][0] - self.kalman_filters_leg[rows[columns.index(i)]].x[0])*10,  (frame[i][1] - self.kalman_filters_leg[rows[columns.index(i)]].x[2])*10])

						_x_updated = self.kalman_filters_leg[rows[columns.index(i)]].x
						# track_KF_point_new.append([_x_updated[0], _x_updated[2]])
						track_KF_point_new.append([_x_updated[0], _x_updated[2],_x_updated[1], _x_updated[3]])

						track_KF_new.append(self.track_KF_leg[rows[columns.index(i)]])
						track_conf_new.append(self.track_conf_leg[rows[columns.index(i)]]*DECAY_RATE_LEG + frame[i][2]*(1-DECAY_RATE_LEG))
						track_size_new.append(self.track_size_leg[rows[columns.index(i)]])

				# # Maintain unassinged KF tracks
				# if len(rows) < len(track_KF):
					# if len(columns) < len(rows):
					# print 'got orphaned KF...'
				for kf_obji in self.track_KF_leg:
					if kf_obji not in track_KF_new:
						_index = self.track_KF_leg.index(kf_obji)
						_conf = self.track_conf_leg[_index]*DECAY_RATE_LEG
						if _conf > DECAY_THRES_LEG:
							# print 'maintaining', kf_obji
							kalman_filters_new.append(self.kalman_filters_leg[_index])
							track_conf_new.append(_conf)
							track_size_new.append(self.track_size_leg[_index])
							# track_KF_point_new.append([track_KF_point[_index][0],track_KF_point[_index][1]])
							track_KF_point_new.append([self.track_KF_point_leg[_index][0],self.track_KF_point_leg[_index][1], self.track_KF_point_leg[_index][2] ,self.track_KF_point_leg[_index][3]])
							# track_new.append(kf_obji)
							track_KF_new.append(kf_obji)


				self.kalman_filters_leg = kalman_filters_new
				self.track_KF_leg = track_KF_new
				# track = track_new
				self.track_conf_leg = track_conf_new
				self.track_size_leg = track_size_new
				self.track_KF_point_leg = track_KF_point_new

				# print track, track_conf
				# # print frame, track_KF_point
				# tracks.append(track)
				# tracks_KF.append(track_KF)
				# tracks_conf.append(track_conf)
				# tracks_KF_points.append(track_KF_point)

				# last_frame_idx = _frame_idx

				# self.display.update(frame, self.track_KF_point_leg)
			self.findPeopleTracks()
			self.processMunkresKalmanPeople()
			self.publishPersons()
			self.publishTracksMarker()
			if self.use_display:
				if not self._first_leg:
					self.display.setup_plot(frame, self.track_KF_point_leg, self.track_KF_point_people, self.track_KF_people, self.track_KF_confirmations, self.track_KF_onelegmode)
				else:
					self.display.update(frame, self.track_KF_point_leg, self.track_KF_point_people, self.track_KF_people, self.track_KF_confirmations, self.track_KF_onelegmode)
		else:
			# print 'Skipping frame %d, empty data, may not real time' % (_frame_idx)
			if self.use_decay_when_nodata:
				if self.use_display:
					pass
					# print 'Skipping frame ..., empty data, may not real time'
					# print 'Decaying all tracks !!, no data'
				track_KF_new = []
				track_conf_new = []
				track_KF_point_new = []
				kalman_filters_new = []
				for kf_obji in self.track_KF_leg:
					_index = self.track_KF_leg.index(kf_obji)
					_conf = self.track_conf_leg[_index]*DECAY_RATE_LEG
					if _conf > DECAY_THRES_LEG:
						kalman_filters_new.append(self.kalman_filters_leg[_index])
						track_conf_new.append(_conf)
						# print self.track_KF_leg, self.track_KF_point_leg
						track_KF_point_new.append([self.track_KF_point_leg[_index][0],self.track_KF_point_leg[_index][1], self.track_KF_point_leg[_index][2] ,self.track_KF_point_leg[_index][3]])
						track_KF_new.append(kf_obji)
				self.kalman_filters_leg = kalman_filters_new
				self.track_KF_leg = track_KF_new
				self.track_conf_leg = track_conf_new
				self.track_KF_point_leg = track_KF_point_new

				self.findPeopleTracks()
				self.processMunkresKalmanPeople()
				self.publishPersons()
				self.publishTracksMarker()
				if self.use_display:
					if not self._first_leg:
						self.display.setup_plot(frame, self.track_KF_point_leg, self.track_KF_point_people, self.track_KF_people, self.track_KF_confirmations, self.track_KF_onelegmode)
					else:
						self.display.update(frame, self.track_KF_point_leg, self.track_KF_point_people, self.track_KF_people, self.track_KF_confirmations, self.track_KF_onelegmode)
			else:
				if self.use_display:
					pass
					#print 'Skipping frame ..., empty data, may not real time'


		# _frame_idx = _frame_idx + 1

		# print tracks
		# return tracks, _obj_id-1, tracks_KF_points, tracks_conf
	pass

	def findPeopleTracks(self): #, leg_confs):
		# people_tracks = []
		# twolegs_tracks = []
		# onelegs_tracks = []

		if self.use_raw_leg_data:
			track = self.points
			conf = [c[2] for c in track]
		else:
			track = self.track_KF_point_leg
			conf  = self.track_conf_leg
			size  = self.track_size_leg
		# for track, conf in zip (leg_tracks, leg_confs):
		if len(track) > 1:
			leg_dists = []
			uniqueness = [0]*(len(track))
			for leg in track:
				leg_dist = []
				leg_index = []
				t_min_dist = 9999.
				t_min_index = 1
				# uniqueness = np.zeros(len(track))

				for oleg in track:
					if track.index(oleg) != track.index(leg):
						_dd = math.hypot(leg[0]-oleg[0],leg[1]-oleg[1])
						if _dd <= PERSON_GATING_DISTANCE:
							leg_dist.append(_dd)
							leg_index.append(track.index(oleg))
							if t_min_dist> _dd:
								t_min_index = track.index(oleg)
								t_min_dist = _dd

				# print leg_index
				if leg_index:
					# print uniqueness
					uniqueness[t_min_index] = uniqueness[t_min_index] + 1
					uniqueness[track.index(leg)] = uniqueness[track.index(leg)] + 1
					leg_dists.append([leg_dist, leg_index, t_min_index])
					# print 'got potential pair', t_min_index, track.index(leg)

					# print uniqueness
				else:
					leg_dists.append([leg_dist, leg_index, -1])
			# if max(uniqueness) > 1:
				# print 'solving conflicting legs... (WARN: For NOW REMOVED!!!)'
			# print uniqueness
			# print leg_dists
			twolegs_track = []
			onelegs_track = []
			t_index = 0
			for uni in uniqueness:
				# t_index = uniqueness.index(uni)
				# print t_index, uni
				if uni == 2:
					# print leg_dists
					t_index2 = leg_dists[t_index][2]#[t_index ]
					# twolegs_tracks.append([leg_dists[-1][2][t_index ],t_index ])
					if uniqueness[t_index2] is not None and t_index2 >= 0:
						_x_index = track[t_index][0]
						_y_index = track[t_index][1]
						_x_index2= track[t_index2][0]
						_y_index2= track[t_index2][1]
						_conf = ( conf[t_index] + conf[t_index2] ) /2
						_size = ( size[t_index] + size[t_index2] ) /2
						twolegs_track.append([t_index2,t_index , _x_index, _y_index, _x_index2, _y_index2, (_x_index + _x_index2)/2, (_y_index + _y_index2)/2, _conf, _size])
						uniqueness[t_index] = None
						uniqueness[t_index2] = None
					else:
						uniqueness[t_index] = None
						onelegs_track.append([t_index, track[t_index][0], track[t_index][1], conf[t_index], size[t_index]])
						# removing already added (conflicting found later)
						tt_idx = 0
						for _twoleg in twolegs_track:
							# if _twoleg[0] == t_index or _twoleg[0] == t_index2 or _twoleg[1] == t_index or _twoleg[1] == t_index2:
							if _twoleg[0] == t_index:
								twolegs_track.pop(twolegs_track.index(_twoleg))
								onelegs_track.append([_twoleg[1], track[_twoleg[1]][0], track[_twoleg[1]][1], conf[_twoleg[1]], size[_twoleg[1]]])
							if _twoleg[1] == t_index:
								twolegs_track.pop(twolegs_track.index(_twoleg))
								onelegs_track.append([_twoleg[0], track[_twoleg[0]][0], track[_twoleg[0]][1], conf[_twoleg[0]], size[_twoleg[1]]])
							tt_idx = tt_idx + 1
				else:
					if uni is not None:
						onelegs_track.append([t_index, track[t_index][0], track[t_index][1], conf[t_index], size[t_index]])
				t_index = t_index + 1
			# print uniqueness
			# print onelegs_track

			# twolegs_tracks.append(twolegs_track)
			# onelegs_tracks.append(onelegs_track)
			self.twolegs_track = twolegs_track
			self.onelegs_track = onelegs_track

		else:
			# no pairing possible
			# people_tracks.append([])

			# twolegs_tracks.append([])
			self.twolegs_track = []
			# onelegs_tracks.append([[0, track[0][0], track[0][1]]])
			try:
				self.onelegs_track = [[0, track[0][0], track[0][1], conf[0], size[0]]]
			except:
				self.onelegs_track = []
		# if self.use_display:
				# print self.twolegs_track
				# print self.onelegs_track
		# return twolegs_tracks, onelegs_tracks

	def processMunkresKalmanPeople(self):
		# kalman_filters = []
		# tracks_KF = []
		# tracks_conf = []
		# tracks_KF_points = []
		# _frame_idx = 0
		# createPersonKF(0,0)

		twolegs_track = self.twolegs_track
		onelegs_track = self.onelegs_track
		# for twolegs_track, onelegs_track in zip (twolegs_tracks, onelegs_tracks):
		if not self._first_people:
			# _first = True
			track_KF = []
			track_conf = []
			track_size = []
			track_KF_point = []
			track_KF_confirmations = []
			track_KF_improvements = []
			track_KF_onelegmode = []
			# _person_id = 1
			if len(twolegs_track) > 0:
				self._first_people = True
				for twoleg in twolegs_track:
					track_KF_onelegmode.append(0)
					track_KF.append(self._people_id)
					self.kalman_filters_people.append(createPersonKF(twoleg[6], twoleg[7], self.KF_DT, self.KF_DT2, KF_pd=PERSON_KF_pd, KF_pv = PERSON_KF_pv, KF_pa = PERSON_KF_pa, KF_rd = PERSON_KF_rd, KF_rv = PERSON_KF_rv, KF_ra = PERSON_KF_ra, KF_q = PERSON_KF_q))
					track_conf.append(twoleg[8])
					track_size.append(twoleg[9])
					track_KF_point.append([twoleg[6], twoleg[7], twoleg[2] , twoleg[3], twoleg[4], twoleg[5]])
					track_KF_improvements.append(twoleg[8]*IMPROVE_RATE)
					track_KF_confirmations.append(False)
					self.generateNewPersonId()

			# else:
			# tracks_KF.append(track_KF)
			# tracks_conf.append(track_conf)
			# tracks_KF_points.append(track_KF_point)
			self.track_KF_people = track_KF
			self.track_conf_people = track_conf
			self.track_size_people = track_size
			self.track_KF_point_people = track_KF_point
			self.track_KF_improvements = track_KF_improvements
			self.track_KF_confirmations = track_KF_confirmations
			self.track_KF_onelegmode = track_KF_onelegmode

		else:
			track_KF_point_new = []
			_kidx = 0
			for kf in self.kalman_filters_people:
				_x_bef_updated = kf.x
				if self.no_ppl_predict_when_update_fail:
					if self.track_KF_onelegmode[_kidx] < 2:
						if self.track_KF_confirmations[_kidx]:
							kf.predict()
				else:
					if self.track_KF_confirmations[_kidx]:
						kf.predict()
				_x_updated = kf.x
				if self.use_limit_ppl_predict:
					_vector_norm = math.hypot(_x_updated[0]-_x_bef_updated[0], _x_updated[3]-_x_bef_updated[3])
					if _vector_norm > LIMIT_PPL_PREDICT:
						_x_updated[0] = _x_bef_updated[0] + (_x_updated[0]-_x_bef_updated[0])*LIMIT_PPL_PREDICT/_vector_norm
						_x_updated[3] = _x_bef_updated[3] + (_x_updated[3]-_x_bef_updated[3])*LIMIT_PPL_PREDICT/_vector_norm
						kf.x[0] = _x_updated[0]
						kf.x[3] = _x_updated[3]
				track_KF_point_new.append([_x_updated[0], _x_updated[3], self.track_KF_point_people[_kidx][2]
					,self.track_KF_point_people[_kidx][3]
					,self.track_KF_point_people[_kidx][4]
					,self.track_KF_point_people[_kidx][5]
					])
				_kidx = _kidx + 1
			# track_KF_point = track_KF_point_new # BUGGISH!!!
			self.track_KF_point_people= track_KF_point_new

			cost_matrix = []
			_lidx = 0

			# Here we need to prioritize twolegs from onelegs.
			# If any twolegs found in the Euclidean radius, no onelegs will be proposed for association
			# else onelegs will be tried, infuture JPDAF will be preferred, as onelegs can be highly spurious
			# But, here in the paper, we prefer smoothing later to remove false alarms
			# NOTE: Euclidean distance can be replaced with Mahalanobis
			# REVISE: twolegs using same GNN approach using Mahalanobis distance


			# print self.track_KF_people
			for objid in self.track_KF_people:
				cost_matrix.append([])
				V = np.array([[self.kalman_filters_people[_lidx].P[0,0],self.kalman_filters_people[_lidx].P[0,3]],[self.kalman_filters_people[_lidx].P[3,0],self.kalman_filters_people[_lidx].P[3,3]]])
				V = V + np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]])
				for leg in twolegs_track:
					# _edist = math.hypot(self.track_KF_point_people[_lidx][0] - leg[6],self.track_KF_point_people[_lidx][1] - leg[7])
					_mdist = mahalanobis(np.array([self.track_KF_point_people[_lidx][0], self.track_KF_point_people[_lidx][1]]),
							np.array([leg[6],leg[7]]),
							np.linalg.inv(V))
					cost_matrix[-1].append(_mdist)
					# cost_matrix[-1].append(_edist)
				_lidx = _lidx + 1

			track_KF_new = []
			track_conf_new = []
			track_size_new = []
			track_KF_point_new = []
			kalman_filters_new = []
			track_KF_confirmations_new = []
			track_KF_improvements_new = []
			track_KF_onelegmode_new = []
			rows = []
			columns = []
			indexes = []

			if _lidx > 0:
				cost_matrix = squareMatrix(cost_matrix, COST_MAX_GATING)
				indexes = self.munkres.compute(cost_matrix)
			for row, column in indexes:
				rows.append(row)
				columns.append(column)

			for i in range(len(twolegs_track)):
				if i not in columns or cost_matrix[rows[columns.index(i)]][i] >= COST_MAX_GATING:
					kalman_filters_new.append(createPersonKF(twolegs_track[i][6], twolegs_track[i][7], self.KF_DT, self.KF_DT2, KF_pd=PERSON_KF_pd, KF_pv = PERSON_KF_pv, KF_pa = PERSON_KF_pa, KF_rd = PERSON_KF_rd, KF_rv = PERSON_KF_rv, KF_ra = PERSON_KF_ra, KF_q = PERSON_KF_q))
					# track_KF_point_new.append([twolegs_track[i][6], twolegs_track[i][7]])
					track_KF_point_new.append([twolegs_track[i][6],twolegs_track[i][7], twolegs_track[i][2] , twolegs_track[i][3], twolegs_track[i][4], twolegs_track[i][5]])
					track_KF_new.append(self._people_id)
					track_conf_new.append(twolegs_track[i][8])
					track_size_new.append(twolegs_track[i][9])
					track_KF_improvements_new.append(twolegs_track[i][8]*IMPROVE_RATE)
					track_KF_confirmations_new.append(False)
					track_KF_onelegmode_new.append(0)
					self.generateNewPersonId()
				else:
					#print 'updated...'
					kalman_filters_new.append(self.kalman_filters_people[rows[columns.index(i)]])
					self.kalman_filters_people[rows[columns.index(i)]].update([twolegs_track[i][6], twolegs_track[i][7]]) 

					_x_updated = self.kalman_filters_people[rows[columns.index(i)]].x
					track_KF_point_new.append([_x_updated[0], _x_updated[3], twolegs_track[i][2]
						, twolegs_track[i][3]
						, twolegs_track[i][4]
						, twolegs_track[i][5]
						])

					track_KF_new.append(self.track_KF_people[rows[columns.index(i)]])
					track_conf_new.append(self.track_conf_people[rows[columns.index(i)]]*DECAY_RATE + twolegs_track[i][8]*(1-DECAY_RATE))
					track_size_new.append(self.track_size_people[rows[columns.index(i)]])
					_improve = self.track_KF_improvements[rows[columns.index(i)]] + twolegs_track[i][8]*IMPROVE_RATE
					track_KF_improvements_new.append(_improve)
					track_KF_onelegmode_new.append(0)
					if _improve > PERSON_CONFIRM_THRES:
						track_KF_confirmations_new.append(True)
					else:
						track_KF_confirmations_new.append(False)

			onelegs_track_owned_index = []
			onelegs_track_owning_person_index = []
			_index_kf_obji = 0
			for kf_obji in self.track_KF_people:
				if kf_obji not in track_KF_new:
					_index = self.track_KF_people.index(kf_obji)
					_xx = self.track_KF_point_people[_index][0]
					_yy = self.track_KF_point_people[_index][1]

					_index_mindist = findMinIndexFromOnelegsTrack(_xx , _yy, onelegs_track, MAX_DIST_OWNERSHIP_ONELEG)
					# for one1leg in onelegs_track:
					# _dist = math.hypot(_xx - one1leg[0], _yy - one1leg[1])
					if _index_mindist is not None:
						onelegs_track_owned_index.append(_index_mindist)
						onelegs_track_owning_person_index.append(_index_kf_obji)
					# else:
						# onelegs_track_owned_index.append(None)
						# onelegs_track_owning_person_index.append(0)
				_index_kf_obji = _index_kf_obji + 1
			# onelegs_track_owned_index = list(OrderedDict.fromkeys(onelegs_track_owned_index))



			for kf_obji in self.track_KF_people:
				if kf_obji not in track_KF_new:
					_index = self.track_KF_people.index(kf_obji)
					_check = isThereAnyOneLegAround(self.track_KF_point_people[_index],self.kalman_filters_people[_index], onelegs_track, onelegs_track_owned_index, onelegs_track_owning_person_index, _index)
					# print _check
					if _check[0] or _check[8]:
						# print 'looking for one leg attachment:', _check[0]
						_gated_len = len(_check[0])
						# print _check, _check[0]
						# print onelegs_track
						if _check[0]:
							_gated_sum_conf = sum([onelegs_track[s][3] for s in _check[0]])
							_conf = self.track_conf_people[_index]*DECAY_RATE + (_gated_sum_conf/_gated_len)*(1-DECAY_RATE)
							_size = self.track_size_people[_index]
							if _conf > DECAY_THRES:
								# if kf_obji == 1:
									# print 'object 1 found at least 1 onelegs averaging and go...'
								_R = self.kalman_filters_people[_index].R
								# mx, my, RR = getMMSEOneLegs(_check, onelegs_track, _R, self.track_KF_point_people[_index])
								mx, my, RR = getMMSEOneLegs(_check, onelegs_track, _R, self.track_KF_point_people[_index], getPosPMatrixCAModel(self.kalman_filters_people[_index]))
								self.kalman_filters_people[_index].R = RR
								self.kalman_filters_people[_index].update([mx, my])
								self.kalman_filters_people[_index].R = _R
								kalman_filters_new.append(self.kalman_filters_people[_index])
								track_conf_new.append(_conf)
								track_size_new.append(_size)
								_x_updated = self.kalman_filters_people[_index].x
								track_KF_point_new.append([_x_updated[0],_x_updated[3]
								# track_KF_point_new.append([self.track_KF_point_people[_index][0],self.track_KF_point_people[_index][1]
									,self.track_KF_point_people[_index][2]
									,self.track_KF_point_people[_index][3]
									,self.track_KF_point_people[_index][4]
									,self.track_KF_point_people[_index][5]
									]) #, track_KF_point[_index][2] ,track_KF_point[_index][3]])
								track_KF_new.append(kf_obji)
								track_KF_improvements_new.append(self.track_KF_improvements[_index])
								track_KF_confirmations_new.append(self.track_KF_confirmations[_index])
								track_KF_onelegmode_new.append(1)
						else:
							# _conf = track_conf[_index] #*DECAY_RATE
							# self.kalman_filters_people[_index].update([self.track_KF_point_people[_index][0],self.track_KF_point_people[_index][1]])
							kalman_filters_new.append(self.kalman_filters_people[_index])
							track_conf_new.append(self.track_conf_people[_index])
							track_size_new.append(self.track_size_people[_index])
							track_KF_point_new.append([self.track_KF_point_people[_index][0],self.track_KF_point_people[_index][1]
								,self.track_KF_point_people[_index][2]
								,self.track_KF_point_people[_index][3]
								,self.track_KF_point_people[_index][4]
								,self.track_KF_point_people[_index][5]
								]) #, track_KF_point[_index][2] ,track_KF_point[_index][3]])
							track_KF_new.append(kf_obji)
							track_KF_improvements_new.append(self.track_KF_improvements[_index])
							track_KF_confirmations_new.append(self.track_KF_confirmations[_index])
							track_KF_onelegmode_new.append(2)
							# if kf_obji == 1:
								# print 'object 1 found no (assosiative) oneleg staying...'
								# print track_KF_point_new

					else:
						_conf = self.track_conf_people[_index]*DECAY_RATE
						_size = self.track_size_people[_index]
						if _conf > DECAY_THRES:
							kalman_filters_new.append(self.kalman_filters_people[_index])
							track_conf_new.append(_conf)
							track_size_new.append(_size)
							track_KF_point_new.append([self.track_KF_point_people[_index][0],self.track_KF_point_people[_index][1]
								,self.track_KF_point_people[_index][2]
								,self.track_KF_point_people[_index][3]
								,self.track_KF_point_people[_index][4]
								,self.track_KF_point_people[_index][5]
								]) #, track_KF_point[_index][2] ,track_KF_point[_index][3]])
							track_KF_new.append(kf_obji)
							track_KF_improvements_new.append(self.track_conf_people[_index])
							track_KF_confirmations_new.append(self.track_conf_people[_index])
							track_KF_onelegmode_new.append(3)
							#if kf_obji == 1:
								#print 'object 1 found no oneleg decaying...'
								#print track_KF_point_new


			self.kalman_filters_people= kalman_filters_new
			self.track_KF_people= track_KF_new
			self.track_conf_people= track_conf_new
			self.track_size_people= track_size_new
			self.track_KF_point_people= track_KF_point_new
			self.track_KF_improvements = track_KF_improvements_new
			self.track_KF_confirmations = track_KF_confirmations_new
			self.track_KF_onelegmode = track_KF_onelegmode_new

			# tracks_KF.append(track_KF)
			# tracks_conf.append(track_conf)
			# tracks_KF_points.append(track_KF_point)

		# _frame_idx = _frame_idx + 1
		# print self.track_KF_onelegmode
		# print self.track_KF_people
		# print self.track_KF_point_people

	# return tracks_KF, _person_id-1, tracks_KF_points, tracks_conf

	def publishTracksMarker(self):
		onelegs_marker = Marker()
		twolegs_marker = Marker()
		onelegs_marker.header.frame_id = "laser"
		twolegs_marker.header.frame_id = "laser"
		onelegs_marker.header.stamp = rospy.Time.now()
		twolegs_marker.header.stamp = rospy.Time.now()
		onelegs_marker.ns = "debug_onelegs"
		twolegs_marker.ns = "debug_twolegs"
		onelegs_marker.id = 10
		twolegs_marker.id = 11
		onelegs_marker.pose.orientation.w = 1.0
		twolegs_marker.pose.orientation.w = 1.0
		onelegs_marker.scale.x = 0.1
		twolegs_marker.scale.x = 0.1
		onelegs_marker.scale.y = 0.1
		twolegs_marker.scale.y = 0.1
		onelegs_marker.scale.z = 0.1
		twolegs_marker.scale.z = 0.1
		onelegs_marker.color.r = 0.5
		onelegs_marker.color.g = 0.5
		onelegs_marker.color.b = 0
		onelegs_marker.color.a = .7
		twolegs_marker.color.r = 0
		twolegs_marker.color.g = 0
		twolegs_marker.color.b = 1
		twolegs_marker.color.a = .7
		onelegs_marker.action = Marker.ADD
		twolegs_marker.action = Marker.ADD
		onelegs_marker.type = 8#Marker.CUBE
		twolegs_marker.type = 8#Marker.CUBE
		for track in self.onelegs_track:
			point = Point()
			point.x = track[1]
			point.y = track[2]
			if self.use_odom:
				_xd = [point.x , point.y]
				_xd = self.odomTranslateAndThenRotate(_xd)
				point.x = _xd[0]
				point.y = _xd[1]
			point.z = 0
			onelegs_marker.points.append(point)
		for track in self.twolegs_track:
			point = Point()
			point.x = track[6]
			point.y = track[7]
			if self.use_odom:
				_xd = [point.x , point.y]
				_xd = self.odomTranslateAndThenRotate(_xd)
				point.x = _xd[0]
				point.y = _xd[1]
			point.z = 0
			twolegs_marker.points.append(point)
		self.pub_marker.publish(onelegs_marker)
		# print onelegs_marker
		self.pub_marker.publish(twolegs_marker)
		# print twolegs_marker
		
	def publishPersons(self):
		persons = PersonTrackArray()
		persons.header.stamp = rospy.Time.now()
		persons.no_of_persons = len(self.track_KF_people)
		if persons.no_of_persons > 0:
			_idx = 0
			for conf in self.track_conf_people:
				person = PersonTrack()
				person.header.stamp = rospy.Time.now()
				person.ConPerson = conf
				person.AvgSizeLegPerson = self.track_size_people[_idx]
				person.xPerson = self.track_KF_point_people[_idx][0]
				person.yPerson = self.track_KF_point_people[_idx][1]
				if self.use_odom:
					_xd = [person.xPerson , person.yPerson]
					_xd = self.odomTranslateAndThenRotate(_xd)
					person.xPerson = _xd[0]
					person.yPerson = _xd[1]
				person.object_id = self.track_KF_people[_idx]
				persons.Persons.append(person)

				_idx = _idx + 1
		# print persons
		self.pub_persons.publish(persons)

def aggreateCoord(data):
	xs, ys = [], []
	for leg in data:
		xs.append(leg[0])
		ys.append(leg[1])
	return xs, ys

def createIds(xx, yy, ids, cfms, olms, ax):
	index = 0
	texts = []
	for _id in ids:
		if olms[index] == 0:
			if cfms[index]:
				text = ax.text(xx[index], yy[index] + 0.3, str(_id), color='red')
			else:
				text = ax.text(xx[index], yy[index] + 0.3, str(_id))
		elif olms[index] == 1:
			text = ax.text(xx[index], yy[index] + 0.3, str(_id), color='green')
		elif olms[index] == 2:
			text = ax.text(xx[index], yy[index] + 0.3, str(_id), color='blue')
		elif olms[index] == 3:
			text = ax.text(xx[index], yy[index] + 0.3, str(_id), color='yellow')
		texts.append(text)
		index = index + 1
	return texts

# class AnimatedScatter(object):
class AnimatedScatter:
	def __init__(self):
		self.fig, self.ax = plt.subplots()
		self.ax.axis([0, 6, -10, 10])
		self.ax.hold(True)
		# plt.axis([0, 4, -10, 10])
		plt.ion()
		self.fig.canvas.draw()
		plt.show(False)
		# plt.draw()
		# self.setup_plot()
		# self.scat2 =  None
		self.scat = self.ax.scatter([], [], c='blue', cmap=plt.cm.PuOr, s=128)
		self.scat2 = self.ax.scatter([], [], c='red', cmap=plt.cm.coolwarm, s=256, marker='+', linewidth=2)
		self.scat3 = self.ax.scatter([], [], c='black', marker='^', s=100, linewidth=4)
		self.texts = []

	def removeTexts(self):
		for text in self.texts:
			text.remove()
		self.texts = []

	def setup_plot(self, data, data_kf, data_pp_ppl, data_kf_ppl, data_kf_ppl_cfm, data_kf_ppl_olm):
		xs, ys = aggreateCoord(data)
		xskf, yskf = aggreateCoord(data_kf)
		xskfppl , yskfppl = aggreateCoord(data_pp_ppl)
		self.removeTexts()
		self.texts = createIds(xskfppl, yskfppl, data_kf_ppl, data_kf_ppl_cfm, data_kf_ppl_olm, self.ax)
		# self.scat = self.ax.scatter(xs, ys, c='blue', cmap=plt.cm.PuOr, s=128)
		self.scat.set_offsets(np.column_stack((xs, ys)))
		self.scat2.set_offsets(np.column_stack((xskf, yskf)))
		self.scat3.set_offsets(np.column_stack((xskfppl, yskfppl)))
		# self.scat2 = self.ax.scatter(xkf, ykf, c=ct, animated=True, cmap=plt.cm.coolwarm, s=256, marker='+', linewidth=2)
		# plt.scatter(xs, ys, c='blue', cmap=plt.cm.PuOr, s=128)
		self.fig.canvas.draw()
		# plt.draw()

	def update(self,data, data_kf, data_pp_ppl, data_kf_ppl, data_kf_ppl_cfm, data_kf_ppl_olm):
		xs, ys = aggreateCoord(data)
		xskf, yskf = aggreateCoord(data_kf)
		xskfppl , yskfppl = aggreateCoord(data_pp_ppl)
		self.removeTexts()
		self.texts = createIds(xskfppl, yskfppl, data_kf_ppl, data_kf_ppl_cfm, data_kf_ppl_olm, self.ax)
		# plt.scatter(xs, ys, c='blue', cmap=plt.cm.PuOr, s=128)
		self.scat.set_offsets(np.column_stack((xs, ys)))
		self.scat2.set_offsets(np.column_stack((xskf, yskf)))
		self.scat3.set_offsets(np.column_stack((xskfppl, yskfppl)))
		# self.scat.set_color(data[2])
		self.fig.canvas.draw()
		# plt.draw()

CLIP_Y_MIN = 0 #1. #0.5
CLIP_Y_MAX = 6 #3.5
CLIP_X_ABS =10 
def clipPoints(frame, abs_max_x, max_y):
	clipped_points = []
	# for frame in points:
		# clipped_points.append([])
	for point in frame:
		if point[1] > -abs_max_x and point[1] < abs_max_x and point[0] > CLIP_Y_MIN  and point[0] < max_y:
			clipped_points.append(point)
	return clipped_points

# display_tracker= AnimatedScatter()
# people_tracker = PeopleTrackerFromLegs(display_tracker)
display_tracker = None
people_tracker = None
def processLegArray(msg):
	# print msg
	tic = time.time()
	points = []
	for l in msg.Legs:
		# print l
		if l.xLeg == 0 and l.yLeg == 0:
			continue
		points.append([l.xLeg, l.yLeg, l.ConLeg, l.SizeLeg])
	if g_use_clip:
		points = clipPoints(points, CLIP_X_ABS , CLIP_Y_MAX)
	people_tracker.processMunkresKalman(points)
	# people_tracker.findPeopleTracks()
	toc = time.time()
	if g_use_display:
		pass
		#print toc - tic


def talker():

	rospy.init_node('track_ped', anonymous=False)
	# parameters exposed
	global COST_MAX_GATING, COST_MAX_GATING_ONELEG, DECAY_RATE, DECAY_THRES, DECAY_RATE_LEG, DECAY_THRES_LEG, IMPROVE_RATE, PERSON_CONFIRM_THRES, RMAHALANOBIS, MAX_DIST_PERSON_ONELEG, PERSON_GATING_DISTANCE, MAX_DIST_OWNERSHIP_ONELEG, LIMIT_PPL_PREDICT, SAMPLING_RATE, MAX_OBJ_ID, g_use_odom, g_odom_topic, g_use_limit_ppl_predict
	global PERSON_KF_ra, PERSON_KF_rd, PERSON_KF_rv, PERSON_KF_q, PERSON_KF_pa, PERSON_KF_pd, PERSON_KF_pv
	global LEG_KF_q, LEG_KF_pd, LEG_KF_pv, LEG_KF_rd, LEG_KF_rv
	COST_MAX_GATING = rospy.get_param('~cost_max_gating', 1.5) #.8#1.5 #.7 #1.5 #.7 #1.5
	COST_MAX_GATING_ONELEG = rospy.get_param('~cost_max_gating_oneleg', .8) #1.5 #.7 #1.5 #.7 #1.5
	DECAY_RATE = rospy.get_param('~decay_rate', 0.97) #0.95#0.93
	DECAY_THRES = rospy.get_param('~decay_thres', 0.3)
	DECAY_RATE_LEG = rospy.get_param('~decay_rate_leg', .90) #0.8 #93
	DECAY_THRES_LEG = rospy.get_param('~decay_thres_leg', .5) #0.3
	IMPROVE_RATE = rospy.get_param('~improve_rate', 0.05) #0.02 #0.05
	PERSON_CONFIRM_THRES = rospy.get_param('~person_confirm_thres', 0.5)
	RMAHALANOBIS = rospy.get_param('~rmahalanobis', 2.) #.5 #2.5 #2.5
	MAX_DIST_PERSON_ONELEG = rospy.get_param('~max_dist_person_oneleg', 1) #.5 #1 #.3
	PERSON_GATING_DISTANCE = rospy.get_param('~person_gating_distance', 0.8)
	MAX_DIST_OWNERSHIP_ONELEG = rospy.get_param('~max_dist_ownership_oneleg', .5) #.35#.5
	LIMIT_PPL_PREDICT = rospy.get_param('~limit_ppl_predict', .13) #0.015 #.03
	MAX_OBJ_ID = rospy.get_param('~max_id', 64)
	SAMPLING_RATE = rospy.get_param('~sampling_rate', 40)

	person_kf_params = rospy.get_param('~person_kf_params', [.5, .2, .5, .01, .1, 1, .7])
	leg_kf_params = rospy.get_param('~leg_kf_params', [.5, .2, .05, .2,.7])

	g_use_odom = rospy.get_param('~use_odom', False)
	g_odom_topic = rospy.get_param('~odom_topic', None)
	g_use_limit_ppl_predict = rospy.get_param('~use_limit_ppl_predict', True)

	print "sampling_rate: %d max_id: %d"% (SAMPLING_RATE, MAX_OBJ_ID)
	print "detailed parameters:\n cost_max_gating: %.3f\n cost_max_gating_oneleg: %.3f \n decay_rate: %.3f\n decay_thres: %.3f\n decay_rate_leg: %.3f\n decay_thres_leg: %.3f\n improve_rate: %.3f\n person_confirm_thres: %.3f\n rmahalanobis: %.3f\n max_dist_person_oneleg: %.3f\n person_gating_distance: %.3f\n max_dist_ownership_oneleg: %.3f\n limit_ppl_predict: %.3f" %(COST_MAX_GATING, COST_MAX_GATING_ONELEG, DECAY_RATE, DECAY_THRES, DECAY_RATE_LEG, DECAY_THRES_LEG, IMPROVE_RATE, PERSON_CONFIRM_THRES, RMAHALANOBIS, MAX_DIST_PERSON_ONELEG, PERSON_GATING_DISTANCE, MAX_DIST_OWNERSHIP_ONELEG, LIMIT_PPL_PREDICT)
	print " use_odom: %s\n odom_topic: %s\n" % (g_use_odom, g_odom_topic)
	print " use_limit_ppl_predict: %s" % (g_use_limit_ppl_predict)
	print " person_kf_params: %s" % (person_kf_params)
	print " leg_kf_params: %s" % (leg_kf_params)

	PERSON_KF_pd = person_kf_params[0]
	PERSON_KF_pv = person_kf_params[1]
	PERSON_KF_pa = person_kf_params[2]
	PERSON_KF_rd = person_kf_params[3]
	PERSON_KF_rv = person_kf_params[4]
	PERSON_KF_ra = person_kf_params[5]
	PERSON_KF_q  = person_kf_params[6]

	LEG_KF_pd = leg_kf_params[0]
	LEG_KF_pv = leg_kf_params[1]
	LEG_KF_rd = leg_kf_params[2]
	LEG_KF_rv = leg_kf_params[3]
	LEG_KF_q  = leg_kf_params[4]

	global g_pub_ppl, display_tracker, people_tracker, g_pub_marker
	# rospy.Subscriber('/usb_cam/image_raw', Image, filter_points)
	rospy.Subscriber('/legs', LegMeasurementArray, processLegArray)
	g_pub_ppl = rospy.Publisher('/persons', PersonTrackArray, queue_size = 10)
	g_pub_marker = rospy.Publisher('visualization_marker_debug', Marker, queue_size = 10)
# <<<<<<< Updated upstream
	# display_tracker= AnimatedScatter()
	people_tracker = PeopleTrackerFromLegs(display_tracker, g_pub_ppl, g_pub_marker, g_use_display, g_use_decay_when_nodata, g_use_raw_leg_data, g_no_ppl_predict_when_update_fail, g_use_limit_ppl_predict , use_odom = g_use_odom, odom_topic = g_odom_topic)
# =======
	display_tracker= None #AnimatedScatter()
	# people_tracker = PeopleTrackerFromLegs(display_tracker, g_pub_ppl, g_use_display, g_use_decay_when_nodata, g_use_raw_leg_data, g_no_ppl_predict_when_update_fail)
# >>>>>>> Stashed changes
	rospy.spin()
	pass

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

