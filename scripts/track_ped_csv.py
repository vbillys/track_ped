#!/usr/bin/env python
import itertools
from munkres import Munkres
from scipy.spatial.distance import mahalanobis
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise, dot3
# import book_format
# import book_plots
import math
import time

import numpy as np
# from matplotlib.patches import Circle, Rectangle, Polygon, Arrow, FancyArrow
import pylab as plt
# import gh_internal as gh
import matplotlib.cm as cm
import matplotlib.animation as animation

f_handle = open('ped_data.csv','r')
# f_handle = open('ped_data_2.csv','r')
# f_handle = open('ped_data_3.csv','r')
# f_handle = open('ped_data_4.csv','r')
# f_handle = open('ped_data_5.csv','r')




class AnimatedScatter(object):
	def __init__(self, data):
		self.data = data
		self.fig, self.ax = plt.subplots()
		# self.stream = self.data_stream()
		self.stream = (x for x in self.data)
		self.ani = animation.FuncAnimation(self.fig, self.update, interval=100, init_func=self.setup_plot, blit=True, frames=len(data)-1, repeat=False)

	def setup_plot(self):
		# x, y, c, ct, obj_id, xkf, ykf = next(self.stream)
		x, y, c, ct, obj_id, xkf, ykf, twoleg_xs, twoleg_ys, oneleg_xs, oneleg_ys = next(self.stream)
		# self.scat = self.ax.scatter(x, y, c=c, animated=True, s=128)
		# self.scat = self.ax.scatter(x, y, c=c, animated=True, cmap=plt.cm.coolwarm, s=128)
		self.scat = self.ax.scatter(x, y, c=c, animated=True, cmap=plt.cm.PuOr, s=128)
		# self.ax.axis([0, 6, -5, 5])
		self.ax.axis([0, 10, -10, 10])
		self.ax.axis([0, 4, -10, 10])
		if len(ct) > 0:
			self.scat2 = self.ax.scatter(xkf, ykf, c=ct, animated=True, cmap=plt.cm.coolwarm, s=256, marker='+', linewidth=2)
			texts = []
			# _idx= 0
			# # for ix,iy in zip(x, y):
			# for _ct in ct:
				# text = self.ax.text(ix, iy + 0.3, str(obj_id[_idx]))
				# texts.append(text)
				# _idx = _idx + 1
			_idx= 0
			# for ix,iy in zip(xkf, ykf):
			for _ct in ct:
				ix , iy = xkf[_idx] , ykf[_idx]
				text = self.ax.text(ix, iy + 0.3, str(obj_id[_idx]))
				texts.append(text)
				_idx = _idx + 1
			clearables = [self.scat, self.scat2]
			clearables = clearables + texts
			# if len(twoleg_xs) > 0:
			# self.scat3 = self.ax.scatter(twoleg_xs, twoleg_ys, c='black', animated=True, marker='^', s=100, linewidth=4)
			self.scat3 = self.ax.scatter([], [], c='black', animated=True, marker='^', s=100, linewidth=4)
			self.scat4 = self.ax.scatter([], [], c='yellow', animated=True, marker='d', s=60, linewidth=3)
			clearables = clearables + [self.scat3, self.scat4]
			# for _xx, _yy in zip (twoleg_xs, twoleg_ys):
				
			return clearables
		else:
			clearables = self.scat
			return clearables,
		# return self.scat, self.scat2
		return clearables

	def update(self, i):
		data = next(self.stream)
		# self.scat.set_offsets(data[:2, :])
		self.scat.set_offsets(np.column_stack((data[0], data[1])))
		# self.scat._sizes = 300 * abs(data[2])**1.5 + 100
		# print np.array(data[2])
		# self.scat.set_array(np.array(data[2]))
		self.scat.set_color(data[2])
		if len(data[3]) > 0:
			self.scat2.set_color(data[3])
			self.scat2.set_offsets(np.column_stack((data[5], data[6])))
			texts = []
			# _idx= 0
			# for ix,iy in zip(data[0], data[1]):
				# text = self.ax.text(ix, iy + 0.3, str(data[4][_idx]))
				# texts.append(text)
				# _idx = _idx + 1
			_idx= 0
			# for ix,iy in zip(data[5], data[6]):
			for _ct in data[3]:
				ix , iy = data[5][_idx] , data[6][_idx]
				text = self.ax.text(ix, iy + 0.3, str(data[4][_idx]))
				texts.append(text)
				_idx = _idx + 1
		# self.scat.set_color(np.array(data[2]))
		# self.scat.set_facecolor(np.column_stack((data[2], data[2])))
		# self.scat.set_array(np.matrix(data[2]))
		# return self.scat, self.scat2
		# return self.scat,
			clearables = [self.scat, self.scat2]
			clearables = clearables + texts
			if len(data[7]) > 0:
				# self.scat3 = self.ax.scatter(data[8], data[9], c='black', animated=True, marker='O', linewidth=4)
				# print len(data[7]), len(data[8])
				self.scat3.set_offsets(np.column_stack((data[7], data[8])))
				clearables = clearables + [self.scat3]
			if len(data[9]) > 0:
				self.scat4.set_offsets(np.column_stack((data[9], data[10])))
				clearables = clearables + [self.scat4]
			return clearables
		else:
			clearables = self.scat
			return clearables,
		# return self.scat, self.scat2
		return clearables

def plotPoints(data):
	ax = plt.axes()

	# sax = plt.subplots()
	maxlen = len(max(data,key=len))
	points_tracked = []
	points_tracked.append([])
	points_tracked.append([])
	points_tracked.append([])
	colors = cm.rainbow(np.linspace(0, 1, maxlen))

	for i in range(maxlen):
		# points_tracked.append([])
		points2nd_x = []
		points2nd_y = []
		for point_ in data:
			if len(point_)>i:
				points2nd_x.append(point_[i][0])
				points2nd_y.append(point_[i][1])
				points_tracked[2].append(colors[i])
		# points_tracked[i].append(points2nd_x)
		# points_tracked[i].append(points2nd_y)
		points_tracked[0] = points_tracked[0] + points2nd_x
		points_tracked[1] = points_tracked[1] + points2nd_y
		# points_tracked[2] = points_tracked[2] + colors[i]
		# points_tracked[2].append(colors[i])
		# color_tracked.append(colors[i])

	# plt.scatter([x[0][0] for x in data], [x[0][1] for x in data], s=128, c='b')
	# plt.scatter(points2nd_x, points2nd_y, s=128, c='r')
	# for _p, c in zip(points_tracked, colors):
		# plt.scatter(_p[0],_p[1] , color=c, s=128)
	# plt.scatter(points_tracked[0], points_tracked[1] , color=points_tracked[2], s=48)
	# plt.scatter(points_tracked[0], points_tracked[1] , color=points_tracked[2])

	scat = ax.scatter(points_tracked[0], points_tracked[1] , color=points_tracked[2], s=48)

	ax.xaxis.grid(True, which="major", linestyle='dotted')
	ax.yaxis.grid(True, which="major", linestyle='dotted')
	# ax.axis([0, 6, -5, 5])
	ax.axis([0, 10, -10, 10])
	# print points_tracked
	# print colors
	# a = AnimatedScatter(points_tracked)
	# plt.show()

def animatePoints(data, tracks_munkres, max_obj_id, KF_points, twolegs_tracks, onelegs_tracks):
	# print onelegs_tracks
	maxlen = len(max(data,key=len))
	print 'maxlen',maxlen
	colors = cm.rainbow(np.linspace(0, 1, maxlen))
	colors_tracks = cm.rainbow(np.linspace(0, 1, max_obj_id+2))
	# colors = np.linspace(.1, .88, maxlen)
	# colors = np.linspace(-0.02, .02, maxlen)
	# colors = np.random.random(maxlen)
	# colors = [0, 105, 7, 8]
	points_timed = []
	# tracks_timed = []

	# print len(data), len(tracks_munkres)
	_idx = 0
	for frame in data:
		if len(frame)>0:
			points_timed.append([])
			# tracks_timed.append([])
			# points_timed[-1].append([])
			# points_timed[-1].append([])
			# points_timed[-1].append([])
			xs = []
			ys = []
			cs = []
			cst= []
			obj_id = []
			xskf = []
			yskf = []
			twoleg_xs = []
			twoleg_ys = []
			oneleg_xs = []
			oneleg_ys = []
			_i = 0
			for _pp in frame:
				# points_timed[-1][0].append(_pp[0]) # = points_timed[-1][0] + _pp[0]
				# points_timed[-1][1].append(_pp[1]) # = points_timed[-1][1] + _pp[1]
				# points_timed[-1][2].append(colors[_i])
				xs.append(_pp[0]) # = points_timed[-1][0] + _pp[0]
				ys.append(_pp[1]) # = points_timed[-1][1] + _pp[1]
				cs.append(colors[_i])
				# colors_tracks[tracks_munkres[_idx][_i]]
				# print _idx, len(tracks_munkres), len(tracks_munkres[_idx]), _i
				# print tracks_munkres[_idx][_i]
				# print len(colors_tracks), tracks_munkres[_idx][_i]

				_i = _i + 1
			_i = 0
			for _pp in tracks_munkres[_idx]:
				cst.append(colors_tracks[tracks_munkres[_idx][_i]])
				xskf.append(KF_points[_idx][_i][0])
				yskf.append(KF_points[_idx][_i][1])
				obj_id.append(tracks_munkres[_idx][_i])
				_i = _i + 1

			for _pp in twolegs_tracks[_idx]:
				# print _pp
				twoleg_xs.append(_pp[6])
				twoleg_ys.append(_pp[7])
			for _pp in onelegs_tracks[_idx]:
				# if _pp:
				print _pp
				oneleg_xs.append(_pp[1])
				oneleg_ys.append(_pp[2])

			# points_timed[-1] = (xs, ys, cs, cst, obj_id, xskf, yskf)
			points_timed[-1] = (xs, ys, cs, cst, obj_id, xskf, yskf, twoleg_xs, twoleg_ys, oneleg_xs, oneleg_ys)
			# tracks_timed[-1] = (xs, ys, cst)
			_idx = _idx + 1

	# print points_timed

	a = AnimatedScatter(points_timed)
	plt.show()

# class KalmanFilterWithConf(KalmanFilter):

def createKF(x,y):
	# kalman_filter = KalmanFilter(dim_x=4, dim_z=2)
	kalman_filter = KalmanFilter(dim_x=4, dim_z=4)
	dt = .1
	KF_F = np.array([[1., dt, 0 ,  0],
		[0 , 1., 0 ,  0],
		[0 , 0 , 1., dt],
		[0 , 0 , 0 , 1.]])
	KF_q = 0.3
	KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(2, dt=0.1, var=KF_q),np.zeros((2,2)))),np.hstack((np.zeros((2,2)),Q_discrete_white_noise(2, dt=0.1, var=KF_q)))))
	KF_pd = 25.
	KF_pv = 10.
	KF_P = np.diag([KF_pd, KF_pv,KF_pd, KF_pv])
	KF_rd = 0.05
	KF_rv = 0.5
	# KF_R = np.diag([KF_rd,KF_rd])
	KF_R = np.diag([KF_rd,KF_rd, KF_rv, KF_rv])
	# KF_H = np.array([[1.,0,0,0],[0,0,1.,0]])
	KF_H = np.array([[1.,0,0,0],[0,0,1.,0],[0,1.,0,0],[0,0,0,1.]])

	kalman_filter.x = np.array([x,0,y,0])
	kalman_filter.F = KF_F
	kalman_filter.H = KF_H
	kalman_filter.Q = KF_Q
	kalman_filter.B = 0
	kalman_filter.R = KF_R
	kalman_filter.P = KF_P

	return kalman_filter

def squareMatrix(mat, fillconstant):
	nrow , ncolumn = len(mat), len(mat[0])
	newmat = mat
	if nrow < ncolumn:
		for i in range(ncolumn - nrow):
			newmat.append([fillconstant]*ncolumn)
	elif ncolumn > nrow:
		for i in range(nrow - ncolumn):
			for _m in newmat:
				_m.append(fillconstant)

	return newmat

COST_MAX_GATING = .8 #1.5 #.7 #1.5 #.7 #1.5
DECAY_RATE = 0.93
DECAY_THRES = 0.3
RMAHALANOBIS = 2. #.5 #2.5 #2.5
def processMunkresKalman(points):

	kalman_filters = []
	munkres = Munkres()
	_first = False
	tracks = []
	tracks_KF = []
	tracks_conf = []
	tracks_KF_points = []
	_frame_idx = 0
	for frame in points:
		if len(frame)>0:
			if not _first:
				_first = True
				track = []
				track_conf = []
				track_KF_point = []
				_obj_id = 1
				for leg in frame:
					track.append(_obj_id)
					kalman_filters.append(createKF(leg[0], leg[1]))
					track_conf.append(leg[2])
					# track_KF_point.append([leg[0],leg[1]])
					track_KF_point.append([leg[0],leg[1],0 ,0])
					_obj_id = _obj_id + 1
				tracks.append(track)
				tracks_KF.append(track)
				tracks_conf.append(track_conf)
				tracks_KF_points.append(track_KF_point)
				print track
				track_KF = track
				last_frame_idx = _frame_idx
			else:
				track_KF_point_new = []
				for kf in kalman_filters:
					kf.predict()
					_x_updated = kf.x
					# track_KF_point_new.append([_x_updated[0], _x_updated[2]])
					track_KF_point_new.append([_x_updated[0], _x_updated[2],_x_updated[1], _x_updated[3]])
				track_KF_point = track_KF_point_new
				cost_matrix = []
				# for prev_leg in points[last_frame_idx]:
				_lidx = 0
				# no_of_object = 0
				# valid_ids = []
				# valid_lidxs = []
				for objid in track:
					# if objid != 0:
						# valid_ids.append(objid)
						# valid_lidxs.append(_lidx)
					cost_matrix.append([])
					# no_of_object = no_of_object + 1
					V = np.array([[kalman_filters[_lidx].P[0,0],kalman_filters[_lidx].P[0,2]],[kalman_filters[_lidx].P[2,0],kalman_filters[_lidx].P[2,2]]])
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
						_mdist = mahalanobis(np.array([track_KF_point[_lidx][0], track_KF_point[_lidx][1]]),
								np.array([leg[0],leg[1]]),
								np.linalg.inv(V))
						# print V, _mdist
						# _dist = math.hypot(points[last_frame_idx][_lidx][0] - leg[0], points[last_frame_idx][_lidx][1] - leg[1])
						_dist = math.hypot(track_KF_point[_lidx][0] - leg[0], track_KF_point[_lidx][1] - leg[1])
						# cost_matrix[-1].append(_dist)
						cost_matrix[-1].append(_mdist)
					_lidx = _lidx + 1
				# print _frame_idx, cost_matrix

				total = 0.
				track_new = []
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
						track_new.append(_obj_id)
						kalman_filters_new.append(createKF(frame[i][0], frame[i][1]))
						track_KF_point_new.append([frame[i][0], frame[i][1]])
						track_KF_new.append(_obj_id)
						track_conf_new.append(frame[i][2])
						_obj_id = _obj_id + 1
					else:
						# print i, columns.index(i), rows[columns.index(i)], cost_matrix
						# if cost_matrix[rows[columns.index(i)]][i] >= COST_MAX_GATING:
							# continue
						# unassigned tracked ids (Munkres) die immediately
						# compute for assigned tracks
						track_new.append(track[rows[columns.index(i)]])
						kalman_filters_new.append(kalman_filters[rows[columns.index(i)]])
						# kalman_filters[rows[columns.index(i)]].update(np.array([frame[i][0], frame[i][1]]))

						# kalman_filters[rows[columns.index(i)]].update([frame[i][0], frame[i][1]])
						kalman_filters[rows[columns.index(i)]].update([frame[i][0], frame[i][1], frame[i][0] - kalman_filters[rows[columns.index(i)]].x[0],  frame[i][1] - kalman_filters[rows[columns.index(i)]].x[2]])

						_x_updated = kalman_filters[rows[columns.index(i)]].x
						# track_KF_point_new.append([_x_updated[0], _x_updated[2]])
						track_KF_point_new.append([_x_updated[0], _x_updated[2],_x_updated[1], _x_updated[3]])

						track_KF_new.append(track[rows[columns.index(i)]])
						track_conf_new.append(track_conf[rows[columns.index(i)]]*DECAY_RATE + frame[i][2]*(1-DECAY_RATE))

				# # Maintain unassinged KF tracks
				# if len(rows) < len(track_KF):
					# if len(columns) < len(rows):
					# print 'got orphaned KF...'
				for kf_obji in track_KF:
					if kf_obji not in track_KF_new:
						_index = track_KF.index(kf_obji)
						_conf = track_conf[_index]*DECAY_RATE
						if _conf > DECAY_THRES:
							# print 'maintaining', kf_obji
							kalman_filters_new.append(kalman_filters[_index])
							track_conf_new.append(_conf)
							# track_KF_point_new.append([track_KF_point[_index][0],track_KF_point[_index][1]])
							track_KF_point_new.append([track_KF_point[_index][0],track_KF_point[_index][1], track_KF_point[_index][2] ,track_KF_point[_index][3]])
							track_new.append(kf_obji)
							track_KF_new.append(kf_obji)


				kalman_filters = kalman_filters_new
				track_KF = track_KF_new
				track = track_new
				track_conf = track_conf_new
				track_KF_point = track_KF_point_new

				print track, track_conf
				# print frame, track_KF_point
				tracks.append(track)
				tracks_KF.append(track_KF)
				tracks_conf.append(track_conf)
				tracks_KF_points.append(track_KF_point)

				last_frame_idx = _frame_idx
		_frame_idx = _frame_idx + 1

	# print tracks
	return tracks, _obj_id-1, tracks_KF_points
	pass

PERSON_GATING_DISTANCE = 0.8
def findPeopleTracks(leg_tracks):
	# people_tracks = []
	twolegs_tracks = []
	onelegs_tracks = []
	for track in leg_tracks:
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
						twolegs_track.append([t_index2,t_index , _x_index, _y_index, _x_index2, _y_index2, (_x_index + _x_index2)/2, (_y_index + _y_index2)/2])
						uniqueness[t_index] = None
						uniqueness[t_index2] = None
					else:
						uniqueness[t_index] = None
						# removing already added (conflicting found later)
						for _twoleg in twolegs_track:
							# if _twoleg[0] == t_index or _twoleg[0] == t_index2 or _twoleg[1] == t_index or _twoleg[1] == t_index2:
							if _twoleg[0] == t_index or _twoleg[1] == t_index:
								twolegs_track.pop(twolegs_track.index(_twoleg))
				else:
					if uni is not None:
						onelegs_track.append([t_index, track[t_index][0], track[t_index][1]])
				t_index = t_index + 1
			# print uniqueness
			# print onelegs_track
			twolegs_tracks.append(twolegs_track)
			onelegs_tracks.append(onelegs_track)

		else:
			# no pairing possible
			# people_tracks.append([])
			twolegs_tracks.append([])
			onelegs_tracks.append([[0, track[0][0], track[0][1]]])
	print twolegs_tracks
	print onelegs_tracks
	return twolegs_tracks, onelegs_tracks



def grouper(n, iterable, fillvalue=None):
	"grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
	args = [iter(iterable)] * n
	return itertools.zip_longest(*args, fillvalue=fillvalue)

f_content = f_handle.readlines()
points = []
for str_ in f_content:
	strs = str_.strip()
	# point_ = strs.split(' ')
	point_ = map(float, strs.split())
	# print point_
	# point = list(grouper(2, point_))
	point = zip(*(iter(point_),) * 3)
	# print point
	points.append(point)
# print points
# tracks_munkres , max_obj_id = processMunkres(points)
t_start = time.time()
tracks_munkres , max_obj_id , tracks_KF_points= processMunkresKalman(points)
people_2legs_tracks, people_1leg_tracks = findPeopleTracks(tracks_KF_points)
t_end = time.time()
print (-t_start + t_end) , len(points), (-t_start + t_end) / len(points)
plotPoints(points)
animatePoints(points, tracks_munkres, max_obj_id, tracks_KF_points, people_2legs_tracks, people_1leg_tracks)
# plotPoints(points_processed)
# animatePoints(points_processed)

# gh.plot_estimate_chart_3()
