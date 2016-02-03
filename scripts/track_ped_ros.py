#!/usr/bin/env python
import rospy
from mech_input.msg import LegMeasurementArray, LegMeasurement #, PersonTrack, PersonTrackArray

from munkres import Munkres
from scipy.spatial.distance import mahalanobis
import math
import time
import numpy as np
import matplotlib
matplotlib.use('GTKAgg')
import pylab as plt
# import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation

from CustomCreateKF import createLegKF, createPersonKF, squareMatrix


COST_MAX_GATING = .8 #1.5 #.7 #1.5 #.7 #1.5
DECAY_RATE = 0.93
DECAY_THRES = 0.3
RMAHALANOBIS = 2. #.5 #2.5 #2.5
PERSON_GATING_DISTANCE = 0.8

class PeopleTrackerFromLegs:
	def __init__(self, display):
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
		# _frame_idx = 0
		self._obj_id = 1
		self.display = display

	def processMunkresKalman(self,points):

		# for frame in points:
		frame = points
		munkres = Munkres()
		# print frame
		if len(frame)>0:
			if not self._first_leg:
				self._first_leg = True
				# track = []
				# track_conf = []
				# track_KF_point = []
				for leg in frame:
					self.track_KF_leg.append(self._obj_id)
					self.kalman_filters_leg.append(createLegKF(leg[0], leg[1]))
					self.track_conf_leg.append(leg[2])
					# self.track_KF_point_leg.append([leg[0],leg[1]])
					self.track_KF_point_leg.append([leg[0],leg[1],0 ,0])
					self._obj_id = self._obj_id + 1
				self.display.setup_plot(frame, self.track_KF_point_leg)
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
				track_KF_point_new = []
				kalman_filters_new = []
				rows = []
				columns = []
				indexes = []

				if _lidx > 0:
					cost_matrix = squareMatrix(cost_matrix, COST_MAX_GATING)
					# print cost_matrix
					indexes = munkres.compute(cost_matrix)

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
						kalman_filters_new.append(createLegKF(frame[i][0], frame[i][1]))
						track_KF_point_new.append([frame[i][0], frame[i][1]])
						track_KF_new.append(self._obj_id)
						track_conf_new.append(frame[i][2])
						self._obj_id = self._obj_id + 1
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
						track_conf_new.append(self.track_conf_leg[rows[columns.index(i)]]*DECAY_RATE + frame[i][2]*(1-DECAY_RATE))

				# # Maintain unassinged KF tracks
				# if len(rows) < len(track_KF):
					# if len(columns) < len(rows):
					# print 'got orphaned KF...'
				for kf_obji in self.track_KF_leg:
					if kf_obji not in track_KF_new:
						_index = self.track_KF_leg.index(kf_obji)
						_conf = self.track_conf_leg[_index]*DECAY_RATE
						if _conf > DECAY_THRES:
							# print 'maintaining', kf_obji
							kalman_filters_new.append(self.kalman_filters_leg[_index])
							track_conf_new.append(_conf)
							# track_KF_point_new.append([track_KF_point[_index][0],track_KF_point[_index][1]])
							track_KF_point_new.append([self.track_KF_point_leg[_index][0],self.track_KF_point_leg[_index][1], self.track_KF_point_leg[_index][2] ,self.track_KF_point_leg[_index][3]])
							# track_new.append(kf_obji)
							track_KF_new.append(kf_obji)


				self.kalman_filters_leg = kalman_filters_new
				self.track_KF_leg = track_KF_new
				# track = track_new
				self.track_conf_leg = track_conf_new
				self.track_KF_point_leg = track_KF_point_new

				# print track, track_conf
				# # print frame, track_KF_point
				# tracks.append(track)
				# tracks_KF.append(track_KF)
				# tracks_conf.append(track_conf)
				# tracks_KF_points.append(track_KF_point)

				# last_frame_idx = _frame_idx

				self.display.update(frame, self.track_KF_point_leg)
			self.findPeopleTracks()
		else:
			# print 'Skipping frame %d, empty data, may not real time' % (_frame_idx)
			print 'Skipping frame ..., empty data, may not real time'

		# _frame_idx = _frame_idx + 1

		# print tracks
		# return tracks, _obj_id-1, tracks_KF_points, tracks_conf
	pass

	def findPeopleTracks(self): #, leg_confs):
		# people_tracks = []
		# twolegs_tracks = []
		# onelegs_tracks = []
		track = self.track_KF_point_leg
		conf  = self.track_conf_leg
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
						twolegs_track.append([t_index2,t_index , _x_index, _y_index, _x_index2, _y_index2, (_x_index + _x_index2)/2, (_y_index + _y_index2)/2, _conf])
						uniqueness[t_index] = None
						uniqueness[t_index2] = None
					else:
						uniqueness[t_index] = None
						onelegs_track.append([t_index, track[t_index][0], track[t_index][1], conf[t_index]])
						# removing already added (conflicting found later)
						tt_idx = 0
						for _twoleg in twolegs_track:
							# if _twoleg[0] == t_index or _twoleg[0] == t_index2 or _twoleg[1] == t_index or _twoleg[1] == t_index2:
							if _twoleg[0] == t_index:
								twolegs_track.pop(twolegs_track.index(_twoleg))
								onelegs_track.append([_twoleg[1], track[_twoleg[1]][0], track[_twoleg[1]][1], conf[_twoleg[1]]])
							if _twoleg[1] == t_index:
								twolegs_track.pop(twolegs_track.index(_twoleg))
								onelegs_track.append([_twoleg[0], track[_twoleg[0]][0], track[_twoleg[0]][1], conf[_twoleg[0]]])
							tt_idx = tt_idx + 1
				else:
					if uni is not None:
						onelegs_track.append([t_index, track[t_index][0], track[t_index][1], conf[t_index]])
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
			self.onelegs_track = [[0, track[0][0], track[0][1]]]
		print self.twolegs_track
		print self.onelegs_track
		# return twolegs_tracks, onelegs_tracks


def aggreateCoord(data):
	xs, ys = [], []
	for leg in data:
		xs.append(leg[0])
		ys.append(leg[1])
	return xs, ys

# class AnimatedScatter(object):
class AnimatedScatter:
	def __init__(self):
		self.fig, self.ax = plt.subplots()
		self.ax.axis([0, 4, -10, 10])
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

	def setup_plot(self, data, data_kf):
		xs, ys = aggreateCoord(data)
		xskf, yskf = aggreateCoord(data_kf)
		# self.scat = self.ax.scatter(xs, ys, c='blue', cmap=plt.cm.PuOr, s=128)
		self.scat.set_offsets(np.column_stack((xs, ys)))
		self.scat2.set_offsets(np.column_stack((xskf, yskf)))
		# self.scat2 = self.ax.scatter(xkf, ykf, c=ct, animated=True, cmap=plt.cm.coolwarm, s=256, marker='+', linewidth=2)
		# plt.scatter(xs, ys, c='blue', cmap=plt.cm.PuOr, s=128)
		self.fig.canvas.draw()
		plt.draw()

	def update(self,data, data_kf):
		xs, ys = aggreateCoord(data)
		xskf, yskf = aggreateCoord(data_kf)
		# plt.scatter(xs, ys, c='blue', cmap=plt.cm.PuOr, s=128)
		self.scat.set_offsets(np.column_stack((xs, ys)))
		self.scat2.set_offsets(np.column_stack((xskf, yskf)))
		# self.scat.set_color(data[2])
		self.fig.canvas.draw()
		plt.draw()

display_tracker= AnimatedScatter()
people_tracker = PeopleTrackerFromLegs(display_tracker)
def processLegArray(msg):
	# print msg
	tic = time.time()
	points = []
	for l in msg.Legs:
		# print l
		if l.xLeg == 0 and l.yLeg == 0:
			continue
		points.append([l.xLeg, l.yLeg, l.ConLeg])
	people_tracker.processMunkresKalman(points)
	# people_tracker.findPeopleTracks()
	toc = time.time()
	print toc - tic


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

