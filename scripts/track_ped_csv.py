#!/usr/bin/env python
import itertools
from collections import OrderedDict
from munkres import Munkres
from scipy.spatial.distance import mahalanobis
from scipy.stats import multivariate_normal
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

# f_handle = open('ped_data.csv','r')
# f_handle = open('ped_data_2.csv','r')
# f_handle = open('ped_data_3.csv','r')
# f_handle = open('ped_data_4.csv','r')
# f_handle = open('ped_data_5.csv','r')
# f_handle = open('ped_data_6.csv','r')
# f_handle = open('ped_data_7sim.csv','r')
f_handle = open('ped_data_8sim.csv','r')




class AnimatedScatter(object):
	def __init__(self, data):
		self.data = data
		self.fig, self.ax = plt.subplots()
		# self.stream = self.data_stream()
		self.stream = (x for x in self.data)
		self.ani = animation.FuncAnimation(self.fig, self.update, interval=100, init_func=self.setup_plot, blit=True, frames=len(data)-1, repeat=False)

	def setup_plot(self):
		# x, y, c, ct, obj_id, xkf, ykf = next(self.stream)
		# x, y, c, ct, obj_id, xkf, ykf, twoleg_xs, twoleg_ys, oneleg_xs, oneleg_ys = next(self.stream)
		x, y, c, ct, obj_id, xkf, ykf, twoleg_xs, twoleg_ys, oneleg_xs, oneleg_ys, xp, yp, ppl_id, ppl_cfm = next(self.stream)
		# self.scat = self.ax.scatter(x, y, c=c, animated=True, s=128)
		# self.scat = self.ax.scatter(x, y, c=c, animated=True, cmap=plt.cm.coolwarm, s=128)
		self.scat = self.ax.scatter(x, y, c=c, animated=True, cmap=plt.cm.PuOr, s=128)
		# self.ax.axis([0, 6, -5, 5])
		# self.ax.axis([0, 10, -10, 10])
		self.ax.axis([0, 6, -10, 10])
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

		clearables = []
		if data[13]:
			self.scat3.set_offsets(np.column_stack((data[11], data[12])))
			_idx= 0
			texts = []
			for _ct in data[13]:
				ix , iy = data[11][_idx] , data[12][_idx]
				if data[14][_idx]:
					text = self.ax.text(ix, iy + 0.3, str(data[13][_idx]), color='red')
				else:
					text = self.ax.text(ix, iy + 0.3, str(data[13][_idx]))
				texts.append(text)
				_idx = _idx + 1
			clearables = clearables + [self.scat3] + texts

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
				# text = self.ax.text(ix, iy + 0.3, str(data[4][_idx]))
				# texts.append(text)
				_idx = _idx + 1
		# self.scat.set_color(np.array(data[2]))
		# self.scat.set_facecolor(np.column_stack((data[2], data[2])))
		# self.scat.set_array(np.matrix(data[2]))
		# return self.scat, self.scat2
		# return self.scat,
			# clearables = [self.scat, self.scat2]
			clearables = clearables + [self.scat, self.scat2]
			clearables = clearables + texts
			# if len(data[7]) > 0:
				# # self.scat3 = self.ax.scatter(data[8], data[9], c='black', animated=True, marker='O', linewidth=4)
				# # print len(data[7]), len(data[8])
				# self.scat3.set_offsets(np.column_stack((data[7], data[8])))
				# clearables = clearables + [self.scat3]
			if len(data[9]) > 0:
				self.scat4.set_offsets(np.column_stack((data[9], data[10])))
				clearables = clearables + [self.scat4]
			return clearables
		else:
			# clearables = self.scat
			clearables = clearables + [self.scat]
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

def animatePoints(data, tracks_munkres, max_obj_id, KF_points, twolegs_tracks, onelegs_tracks, people_KF_points, tracks_people, tracks_confirms):
	# print onelegs_tracks
	maxlen = len(max(data,key=len))
	print 'maxlen',maxlen
	print 'twolegs_len ', len(twolegs_tracks), 'tracks_people_len ', len(tracks_people)
	print 'people_KF_points', len(people_KF_points)
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
			xskf_people = []
			yskf_people = []
			people_id = []
			people_confirm = []


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

			_i = 0
			# print _idx
			# print tracks_people
			# print _idx
			try:
				if people_KF_points[_idx]:
					for _pp in tracks_people[_idx]:
						# print tracks_people[_idx]
						# print _pp
						# print len(tracks_people[_idx]), len(people_KF_points[_idx])
						# print people_KF_points[_idx]
						# cst.append(colors_tracks[tracks_munkres[_idx][_i]])
						xskf_people.append(people_KF_points[_idx][_i][0])
						yskf_people.append(people_KF_points[_idx][_i][1])
						people_id.append(tracks_people[_idx][_i])
						people_confirm.append(tracks_confirms[_idx][_i])
						_i = _i + 1
			except:
				pass

			try:
				for _pp in twolegs_tracks[_idx]:
					# print _pp
					twoleg_xs.append(_pp[6])
					twoleg_ys.append(_pp[7])
			except:
				pass
			try:
				for _pp in onelegs_tracks[_idx]:
					# if _pp:
					# print _pp
					oneleg_xs.append(_pp[1])
					oneleg_ys.append(_pp[2])
			except:
				pass

			# points_timed[-1] = (xs, ys, cs, cst, obj_id, xskf, yskf)
			# points_timed[-1] = (xs, ys, cs, cst, obj_id, xskf, yskf, twoleg_xs, twoleg_ys, oneleg_xs, oneleg_ys)
			points_timed[-1] = (xs, ys, cs, cst, obj_id, xskf, yskf, twoleg_xs, twoleg_ys, oneleg_xs, oneleg_ys, xskf_people, yskf_people, people_id, people_confirm)
			# tracks_timed[-1] = (xs, ys, cst)
			_idx = _idx + 1

	# print points_timed

	a = AnimatedScatter(points_timed)
	plt.show()

# class KalmanFilterWithConf(KalmanFilter):

def createPersonKF(x,y):
	kalman_filter = KalmanFilter(dim_x=6, dim_z=2)
	# kalman_filter = KalmanFilter(dim_x=4, dim_z=4)
	dt = .1
	dt2= .005
	# KF_F = np.array([[1., dt, 0 ,  0],
		# [0 , 1., 0 ,  0],
		# [0 , 0 , 1., dt],
		# [0 , 0 , 0 , 1.]])
	KF_Fca = np.array([[1.,dt,dt2/2],[0,1.,dt],[0,0,1]])
	KF_F = np.vstack((np.hstack((KF_Fca, np.zeros((3,3)) )),# np.zeros((3,3)))),
		np.hstack((np.zeros((3,3)), KF_Fca))  )) # , np.zeros((3,3)))) )))#,
		# np.hstack((np.zeros((3,3)),np.zeros((3,3)), KF_Fca))))
	KF_q = 0.7 #0.3
	# KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(2, dt=0.1, var=KF_q),np.zeros((2,2)))),np.hstack((np.zeros((2,2)),Q_discrete_white_noise(2, dt=0.1, var=KF_q)))))
	KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(3, dt=0.1, var=KF_q),np.zeros((3,3)))),np.hstack((np.zeros((3,3)),Q_discrete_white_noise(3, dt=0.1, var=KF_q)))))
	KF_pd = 25.
	KF_pv = 10.
	KF_pa = 30.
	KF_P = np.diag([KF_pd, KF_pv, KF_pa,KF_pd, KF_pv, KF_pa])
	KF_rd = 0.05
	KF_rv = 0.2 #0.5
	KF_ra = 2 #0.5
	KF_R = np.diag([KF_rd,KF_rd])
	# KF_R = np.diag([KF_rd,KF_rd, KF_rv, KF_rv])
	# KF_R = np.diag([KF_rd,KF_rd, KF_rv, KF_rv, KF_ra, KF_ra])
	# KF_H = np.array([[1.,0,0,0],[0,0,1.,0]])
	# KF_H = np.array([[1.,0,0,0],[0,0,1.,0],[0,1.,0,0],[0,0,0,1.]])
	KF_H = np.array([[1.,0,0,0,0,0],[0,0,0,1.,0,0]])

	# kalman_filter.x = np.array([x,0,y,0])
	kalman_filter.x = np.array([x,0,0,y,0,0])
	kalman_filter.F = KF_F
	kalman_filter.H = KF_H
	kalman_filter.Q = KF_Q
	kalman_filter.B = 0
	kalman_filter.R = KF_R
	kalman_filter.P = KF_P

	return kalman_filter

def createLegKF(x,y):
	# kalman_filter = KalmanFilter(dim_x=4, dim_z=2)
	kalman_filter = KalmanFilter(dim_x=4, dim_z=4)
	dt = .1
	KF_F = np.array([[1., dt, 0 ,  0],
		[0 , 1., 0 ,  0],
		[0 , 0 , 1., dt],
		[0 , 0 , 0 , 1.]])
	KF_q = 0.7 #0.3
	KF_Q = np.vstack((np.hstack((Q_discrete_white_noise(2, dt=0.1, var=KF_q),np.zeros((2,2)))),np.hstack((np.zeros((2,2)),Q_discrete_white_noise(2, dt=0.1, var=KF_q)))))
	KF_pd = 25.
	KF_pv = 10.
	KF_P = np.diag([KF_pd, KF_pv,KF_pd, KF_pv])
	KF_rd = 0.05
	KF_rv = 0.2 #0.5
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
COST_MAX_GATING_ONELEG = .8 #1.5 #.7 #1.5 #.7 #1.5
DECAY_RATE = 0.93
DECAY_THRES = 0.3
DECAY_RATE_LEG = 0.8 #93
DECAY_THRES_LEG = .5 #0.3
IMPROVE_RATE = 0.05
PERSON_CONFIRM_THRES = 0.5
RMAHALANOBIS = 2. #.5 #2.5 #2.5
MAX_DIST_PERSON_ONELEG = 1. #.1 #.5 #1. #.3
PERSON_GATING_DISTANCE = .8 #.6 #0.8
MAX_DIST_OWNERSHIP_ONELEG = .5

def getVMatrixCAModel(k_filter):
	V = np.array([[k_filter.P[0,0],k_filter.P[0,3]],[k_filter.P[3,0],k_filter.P[3,3]]])
	# V = V + np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]])
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

def processMunkresKalmanPeople(twolegs_tracks, onelegs_tracks):
	kalman_filters = []
	munkres = Munkres()
	_first = False
	tracks_KF = []
	tracks_conf = []
	tracks_KF_points = []
	_frame_idx = 0
	# createPersonKF(0,0)
	tracks_KF_confirmations = []
	tracks_KF_improvements = []

	for twolegs_track, onelegs_track in zip (twolegs_tracks, onelegs_tracks):
		if not _first:
			# _first = True
			track_KF = []
			track_conf = []
			track_KF_point = []
			track_KF_confirmations = []
			track_KF_improvements = []
			_person_id = 1
			if len(twolegs_track) > 0:
				_first = True
				for twoleg in twolegs_track:
					track_KF.append(_person_id)
					kalman_filters.append(createPersonKF(twoleg[6], twoleg[7]))
					track_conf.append(twoleg[8])
					_person_id = _person_id + 1
					track_KF_point.append([twoleg[6],twoleg[7], twoleg[2] , twoleg[3], twoleg[4], twoleg[5]])
					track_KF_improvements.append(twoleg[8]*IMPROVE_RATE)
					track_KF_confirmations.append(False)

			# else:
			tracks_KF.append(track_KF)
			tracks_conf.append(track_conf)
			tracks_KF_points.append(track_KF_point)
			tracks_KF_improvements.append(track_KF_improvements)
			tracks_KF_confirmations.append(track_KF_confirmations)

		else:
			track_KF_point_new = []
			_kidx = 0
			for kf in kalman_filters:
				kf.predict()
				_x_updated = kf.x
				track_KF_point_new.append([_x_updated[0], _x_updated[3], track_KF_point[_kidx][2]
					,track_KF_point[_kidx][3]
					,track_KF_point[_kidx][4]
					,track_KF_point[_kidx][5]
					])
				_kidx = _kidx + 1
			track_KF_point = track_KF_point_new
			cost_matrix = []
			_lidx = 0

			# Here we need to prioritize twolegs from onelegs.
			# If any twolegs found in the Euclidean radius, no onelegs will be proposed for association
			# else onelegs will be tried, infuture JPDAF will be preferred, as onelegs can be highly spurious
			# But, here in the paper, we prefer smoothing later to remove false alarms
			# NOTE: Euclidean distance can be replaced with Mahalanobis
			# REVISE: twolegs using same GNN approach using Mahalanobis distance

				
			for objid in track_KF:
				cost_matrix.append([])
				V = np.array([[kalman_filters[_lidx].P[0,0],kalman_filters[_lidx].P[0,3]],[kalman_filters[_lidx].P[3,0],kalman_filters[_lidx].P[3,3]]])
				V = V + np.array([[RMAHALANOBIS,0],[0,RMAHALANOBIS]])
				for leg in twolegs_track:
					_dist = math.hypot(track_KF_point[_lidx][0] - leg[6], track_KF_point[_lidx][1] - leg[7])
					# _mdist = mahalanobis(np.array([track_KF_point[_lidx][0], track_KF_point[_lidx][1]]),
							# np.array([leg[6],leg[7]]),
							# np.linalg.inv(V))
					# cost_matrix[-1].append(_mdist)
					cost_matrix[-1].append(_dist)
				_lidx = _lidx + 1

			track_KF_new = []
			track_conf_new = []
			track_KF_point_new = []
			kalman_filters_new = []
			track_KF_confirmations_new = []
			track_KF_improvements_new = []
			rows = []
			columns = []

			if _lidx > 0:
				cost_matrix = squareMatrix(cost_matrix, COST_MAX_GATING)
				indexes = munkres.compute(cost_matrix)
			for row, column in indexes:
				rows.append(row)
				columns.append(column)

			for i in range(len(twolegs_track)):
				if i not in columns or cost_matrix[rows[columns.index(i)]][i] >= COST_MAX_GATING:
					kalman_filters_new.append(createPersonKF(twolegs_track[i][6], twolegs_track[i][7]))
					# track_KF_point_new.append([twolegs_track[i][6], twolegs_track[i][7]])
					track_KF_point_new.append([twolegs_track[i][6],twolegs_track[i][7], twolegs_track[i][2] , twolegs_track[i][3], twolegs_track[i][4], twolegs_track[i][5]])
					track_KF_new.append(_person_id)
					track_conf_new.append(twolegs_track[i][8])
					track_KF_improvements_new.append(twolegs_track[i][8]*IMPROVE_RATE)
					track_KF_confirmations_new.append(False)
					_person_id = _person_id + 1
				else:
					kalman_filters_new.append(kalman_filters[rows[columns.index(i)]])
					kalman_filters[rows[columns.index(i)]].update([twolegs_track[i][6], twolegs_track[i][7]]) 

					_x_updated = kalman_filters[rows[columns.index(i)]].x
					track_KF_point_new.append([_x_updated[0], _x_updated[3], twolegs_track[i][2]
						, twolegs_track[i][3]
						, twolegs_track[i][4]
						, twolegs_track[i][5]
						])

					track_KF_new.append(track_KF[rows[columns.index(i)]])
					track_conf_new.append(track_conf[rows[columns.index(i)]]*DECAY_RATE + twolegs_track[i][8]*(1-DECAY_RATE))
					_improve = track_KF_improvements[rows[columns.index(i)]] + twolegs_track[i][8]*IMPROVE_RATE
					track_KF_improvements_new.append(_improve)
					if _improve > PERSON_CONFIRM_THRES:
						track_KF_confirmations_new.append(True)
					else:
						track_KF_confirmations_new.append(False)


			onelegs_track_owned_index = []
			onelegs_track_owning_person_index = []
			_index_kf_obji = 0
			for kf_obji in track_KF:
				if kf_obji not in track_KF_new:
					_index = track_KF.index(kf_obji)
					_xx = track_KF_point[_index][0]
					_yy = track_KF_point[_index][1]

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


			for kf_obji in track_KF:
				if kf_obji not in track_KF_new:
					_index = track_KF.index(kf_obji)
					_check = isThereAnyOneLegAround(track_KF_point[_index],kalman_filters[_index], onelegs_track, onelegs_track_owned_index, onelegs_track_owning_person_index, _index)
					if _check[0] or _check[8]:
						_gated_len = len(_check[0])
						print _check, _check[0]
						print onelegs_track
						# if not _check[8]:
						if _check[0]:
							_gated_sum_conf = sum([onelegs_track[s][3] for s in _check[0]])
							_conf = track_conf[_index]*DECAY_RATE + (_gated_sum_conf/_gated_len)*(1-DECAY_RATE)
							if _conf > DECAY_THRES:
								_R = kalman_filters[_index].R
								mx, my, RR = getMMSEOneLegs(_check, onelegs_track, _R, track_KF_point[_index], getPosPMatrixCAModel(kalman_filters[_index]))
								kalman_filters[_index].R = RR
								kalman_filters[_index].update([mx, my])
								kalman_filters[_index].R = _R
								kalman_filters_new.append(kalman_filters[_index])
								track_conf_new.append(_conf)
								_x_updated = kalman_filters[_index].x
								track_KF_point_new.append([_x_updated[0],_x_updated[3]
									,track_KF_point[_index][2]
									,track_KF_point[_index][3]
									,track_KF_point[_index][4]
									,track_KF_point[_index][5]
									]) #, track_KF_point[_index][2] ,track_KF_point[_index][3]])
								track_KF_new.append(kf_obji)
								track_KF_improvements_new.append(track_KF_improvements[_index])
								track_KF_confirmations_new.append(track_KF_confirmations[_index])
						else:
							# _conf = track_conf[_index] #*DECAY_RATE
							kalman_filters_new.append(kalman_filters[_index])
							track_conf_new.append(track_conf[_index])
							track_KF_point_new.append([track_KF_point[_index][0],track_KF_point[_index][1]
								,track_KF_point[_index][2]
								,track_KF_point[_index][3]
								,track_KF_point[_index][4]
								,track_KF_point[_index][5]
								]) #, track_KF_point[_index][2] ,track_KF_point[_index][3]])
							track_KF_new.append(kf_obji)
							track_KF_improvements_new.append(track_KF_improvements[_index])
							track_KF_confirmations_new.append(track_KF_confirmations[_index])

					else:
						_conf = track_conf[_index]*DECAY_RATE
						if _conf > DECAY_THRES:
							kalman_filters_new.append(kalman_filters[_index])
							track_conf_new.append(_conf)
							track_KF_point_new.append([track_KF_point[_index][0],track_KF_point[_index][1]
								,track_KF_point[_index][2]
								,track_KF_point[_index][3]
								,track_KF_point[_index][4]
								,track_KF_point[_index][5]
								]) #, track_KF_point[_index][2] ,track_KF_point[_index][3]])
							track_KF_new.append(kf_obji)
							track_KF_improvements_new.append(track_KF_improvements[_index])
							track_KF_confirmations_new.append(track_KF_confirmations[_index])


			kalman_filters = kalman_filters_new
			track_KF = track_KF_new
			track_conf = track_conf_new
			track_KF_point = track_KF_point_new
			track_KF_confirmations = track_KF_confirmations_new
			track_KF_improvements = track_KF_improvements_new

			tracks_KF.append(track_KF)
			tracks_conf.append(track_conf)
			tracks_KF_points.append(track_KF_point)
			tracks_KF_improvements.append(track_KF_improvements)
			tracks_KF_confirmations.append(track_KF_confirmations)

		_frame_idx = _frame_idx + 1

	return tracks_KF, _person_id-1, tracks_KF_points, tracks_conf, tracks_KF_confirmations

CLIP_Y_MIN = 0 #1. #0.5
CLIP_Y_MAX = 6 #3.5
CLIP_X_ABS =10 
def clipPoints(points, abs_max_x, max_y):
	clipped_points = []
	for frame in points:
		clipped_points.append([])
		for point in frame:
			if point[1] > -abs_max_x and point[1] < abs_max_x and point[0] > CLIP_Y_MIN  and point[0] < max_y:
				clipped_points[-1].append(point)
	return clipped_points


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
					kalman_filters.append(createLegKF(leg[0], leg[1]))
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

						# _dist = math.hypot(track_KF_point[_lidx][0] - leg[0], track_KF_point[_lidx][1] - leg[1])
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
						kalman_filters_new.append(createLegKF(frame[i][0], frame[i][1]))
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
						kalman_filters[rows[columns.index(i)]].update([frame[i][0], frame[i][1], (frame[i][0] - kalman_filters[rows[columns.index(i)]].x[0])*10,  (frame[i][1] - kalman_filters[rows[columns.index(i)]].x[2])*10])

						_x_updated = kalman_filters[rows[columns.index(i)]].x
						# track_KF_point_new.append([_x_updated[0], _x_updated[2]])
						track_KF_point_new.append([_x_updated[0], _x_updated[2],_x_updated[1], _x_updated[3]])

						track_KF_new.append(track[rows[columns.index(i)]])
						track_conf_new.append(track_conf[rows[columns.index(i)]]*DECAY_RATE_LEG + frame[i][2]*(1-DECAY_RATE_LEG))

				# # Maintain unassinged KF tracks
				# if len(rows) < len(track_KF):
					# if len(columns) < len(rows):
					# print 'got orphaned KF...'
				for kf_obji in track_KF:
					if kf_obji not in track_KF_new:
						_index = track_KF.index(kf_obji)
						_conf = track_conf[_index]*DECAY_RATE_LEG
						if _conf > DECAY_THRES_LEG:
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
		else:
			print 'Skipping frame %d, empty data, may not real time' % (_frame_idx)

		_frame_idx = _frame_idx + 1

	# print tracks
	return tracks, _obj_id-1, tracks_KF_points, tracks_conf
	pass

def findPeopleTracks(leg_tracks, leg_confs, points):
	# people_tracks = []
	twolegs_tracks = []
	onelegs_tracks = []
	leg_tracks_ = leg_tracks
	leg_confs_ = leg_confs
	# leg_tracks_ = []
	# leg_confs_ = []
	# for track in points:
		# if len(track) > 1:
			# leg_tracks_.append([])
			# leg_confs_.append([])
			# for ttrack in track:
				# leg_tracks_[-1].append(ttrack[0:2])
				# leg_confs_[-1].append(ttrack[2])
	# print leg_tracks_
	# print leg_confs_
		
	for track, conf in zip (leg_tracks_, leg_confs_):
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
			twolegs_tracks.append(twolegs_track)
			onelegs_tracks.append(onelegs_track)

		else:
			# print track
			# no pairing possible
			# people_tracks.append([])
			twolegs_tracks.append([])
			onelegs_tracks.append([[0, track[0][0], track[0][1], conf[0]]])
	# print twolegs_tracks
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
points = clipPoints(points, CLIP_X_ABS , CLIP_Y_MAX)
tracks_munkres , max_obj_id , tracks_KF_points, tracks_conf= processMunkresKalman(points)
people_2legs_tracks, people_1leg_tracks = findPeopleTracks(tracks_KF_points, tracks_conf, points)
tracks_KF_people, max_people_id, tracks_KF_people_pp, tracks_people_conf, tracks_confirms = processMunkresKalmanPeople(people_2legs_tracks, people_1leg_tracks)
t_end = time.time()
print (-t_start + t_end) , len(points), (-t_start + t_end) / len(points)
plotPoints(points)
# print people_2legs_tracks
# print tracks_KF_people_pp
# print tracks_KF_points
animatePoints(points, tracks_munkres, max_obj_id, tracks_KF_points, people_2legs_tracks, people_1leg_tracks, tracks_KF_people_pp, tracks_KF_people, tracks_confirms)
# plotPoints(points_processed)
# animatePoints(points_processed)

# gh.plot_estimate_chart_3()
