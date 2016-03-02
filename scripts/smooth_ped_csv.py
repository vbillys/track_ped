#!/usr/bin/env python
import itertools
from collections import namedtuple

import math
import time

import numpy as np
import pylab as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import matplotlib.ticker as ticker

import pickle as pl
import brewer2mpl

# brewer2mpl.get_map args: set name  set type  number of colors
bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
g_colors = bmap.mpl_colors

# f_handle = open('processed_data_5.csv','r')
# f_handle = open('processed_data_8sim.csv','r')
f_handle = open('processed_data_1paper.csv','r')
# f_handle = open('processed_data_2paper.csv','r')
# f_handle = open('processed_data_3paper.csv','r')
# f_handle = open('processed_data_4paper.csv','r')
# f_handle = open('processed_data_5paper.csv','r')
# f_handle = open('processed_data_6paper.csv','r')

def namedlist(typename, field_names):
    fields_len = len(field_names)
    fields_text = repr(tuple(field_names)).replace("'", "")[1:-1] # tuple repr without parens or quotes

    class ResultType(list):
	__slots__ = ()
	_fields = field_names

	def _fixed_length_error(*args, **kwargs):
		raise TypeError(u"Named list has fixed length")
	append = _fixed_length_error
	insert = _fixed_length_error
	pop = _fixed_length_error
	remove = _fixed_length_error

	def sort(self):
		raise TypeError(u"Sorting named list in place would corrupt field accessors. Use sorted(x)")

	def _replace(self, **kwargs):
		values = map(kwargs.pop, field_names, self)
		if kwargs:
		    raise TypeError(u"Unexpected field names: {s!r}".format(kwargs.keys()))

		if len(values) != fields_len:
			raise TypeError(u"Expected {e} arguments, got {n}".format(
				    e=fields_len, n=len(values)))

		return ResultType(*values)

	def __repr__(self):
		items_repr=", ".join("{name}={value!r}".format(name=name, value=value)
				for name, value in zip(field_names, self))
		return "{typename}({items})".format(typename=typename, items=items_repr)

    ResultType.__init__ = eval("lambda self, {fields}: self.__setitem__(slice(None, None, None), [{fields}])".format(fields=fields_text))
    ResultType.__name__ = typename

    for i, name in enumerate(field_names):
	fget = eval("lambda self: self[{0:d}]".format(i))
	fset = eval("lambda self, value: self.__setitem__({0:d}, value)".format(i))
	setattr(ResultType, name, property(fget, fset))

    return ResultType

ProcessedData = namedtuple('ProcessedData','people twoleg oneleg targets targets_ids')
TargetData = namedlist('TargetData',('targets', 'targets_frames', 'targets_modes', 'max_target_id'))

def readProcessedData(f_handle):
	f_content = f_handle.readlines()
	# processed_frames = []
	rolling_count = 0
	people = []
	twoleg = []
	oneleg = []
	for str_ in f_content:
		strs = str_.strip()
		point_ = map(float, strs.split())
		if rolling_count % 3 == 0:
			people.append([])
			twoleg.append([])
			oneleg.append([])

		if strs:
			if rolling_count % 3 == 0:
				ppl_ = zip(*(iter(point_),) * 5)
				# people[-1].append(ppl_)
				people[-1] = people[-1] + ppl_
				# twoleg.append([])
				# oneleg.append([])
			elif rolling_count % 3 == 1:
				two_ = zip(*(iter(point_),) * 3)
				# people.append([])
				# twoleg[-1].append(two_)
				twoleg[-1] = twoleg[-1] + two_
				# oneleg.append([])
			elif rolling_count % 3 == 2:
				one_ = zip(*(iter(point_),) * 3)
				# people.append([])
				# twoleg.append([])
				# oneleg[-1].append(one_)
				oneleg[-1] = oneleg[-1] + one_
		# else:
			# people.append([])
			# twoleg.append([])
			# oneleg.append([])
		rolling_count = rolling_count + 1
	return ProcessedData(people, twoleg, oneleg, [], [])

def findMaxTargetId(processed_data):
	max_obj_id = 0
	for frame in processed_data.people:
		if frame:
			ids = [int(ppl[1]) for ppl in frame]
			# for ppl in frame:
				# ids.append(int(ppl[1]))
			max_id = max(ids)
			if max_obj_id < max_id:
				max_obj_id = max_id
	return max_obj_id


def gatherPeopleTracks(processed_data):
	# processed_data.targets = []
	# processed_data.targets_ids = []
	max_obj_id = findMaxTargetId(processed_data)
	_first = True
	_index_frame = 0
	# processed_data.targets.append([[]]*max_obj_id)
	# processed_data.targets_ids.append([[]]*max_obj_id)
	# targets = [[] for _ in range(max_obj_id)]
	# targets_ids = [[] for _ in range(max_obj_id)]

	target_data = TargetData([[] for _ in range(max_obj_id)], [[] for _ in range(max_obj_id)], [[] for _ in range(max_obj_id)], max_obj_id)
	# target_data = dict(targets = {x: [] for x in range (1, max_obj_id+1)}, targets_ids = {x: [] for x in range (1, max_obj_id+1)})
	# for people, twoleg, oneleg in zip(processed_data.people, processed_data.twoleg, processed_data.oneleg):
	for frame in processed_data.people:
		# processed_data.targets_ids = range(1, max_obj_id+1)

		
		# if frame:
		for target in frame:
			# print target
			# print target_data.targets, target_data.targets_ids
			# print int(target[1])
			# print target
			target_data.targets[int(target[1])-1].append([target[2],target[3],target[0]])
			target_data.targets_frames[int(target[1])-1].append(_index_frame)
			target_data.targets_modes[int(target[1])-1].append(int(target[4]))
			# targets[int(target[1])-1].append([target[2],target[3],target[0]])
			# targets_ids[int(target[1])-1].append(_index_frame)
			# print _index_frame
			# if _first:
				# for target in frame:
					# processed_data.targets.append([target[2],target[3],target[0]])
					# processed_data.targets_ids.append([target[1]])
				# _first = False
			# else:
				# if target[1] in processed_data.targets_ids:
		_index_frame = _index_frame + 1
	return target_data
	# return targets, targets_ids

EDYN_WEIGHT = 2 #.03
EEXC_WEIGHT = 1 #.6
EPER_WEIGHT = 1 #.6
EREG_WEIGHT = .5 #.6
EOBS_LAMBDA = .12 #10#.1
TARGET_SIZE = 35 #20
CSIG = TARGET_SIZE * TARGET_SIZE
MIU_EPER = 1

class ContTracking:
	def __init__(self, processed_data, target_data):
		self.processed_data = processed_data
		self.target_data = target_data

	def calcEObs(self):
		fx = 0
		# _index_frame = 0
		# looping summation over all frames (all times)
		# for frame in processed_data.people:
			# # looping summation over all targes in all frames
		_index_target = 0
		# no need to loop over the people detections, we already parsed the targets
		for target in self.target_data.targets:
			# check if the current target is there in the current frame
			# if _index_frame in target_data.targets_frames[_index_target]:
			_index_time = 0
			for _index_frame in self.target_data.targets_frames[_index_target]:
				# print _index_target
				# _target_x = target[target_data.targets_frames[_index_target].index(_index_frame)][0]
				# _target_y = target[target_data.targets_frames[_index_target].index(_index_frame)][1]
				_target_x = target[_index_time][0]
				_target_y = target[_index_time][1]
				# get detections in current frame
				_detections_x = np.array([ x[1] for x in self.processed_data.twoleg[_index_frame]+processed_data.oneleg[_index_frame] ] )
				_detections_y = np.array([ x[2] for x in self.processed_data.twoleg[_index_frame]+processed_data.oneleg[_index_frame] ] )
				# get weights for confidences
				_detections_w = np.array([ x[0] for x in self.processed_data.twoleg[_index_frame]+processed_data.oneleg[_index_frame] ] )
				# get differences per axis x and y and then square them
				# remember we are using cm units here in Energy
				# print target
				_xsx = ((_detections_x - _target_x)*100)**2
				_ysy = ((_detections_y - _target_y)*100)**2
				inner_sum = sum ((CSIG*_detections_w) / (_xsx + _ysy + CSIG))
				# include lambda (this can be affected by occlusion or targets_modes) and the summation result over all detections
				fx = fx + EOBS_LAMBDA - inner_sum
				_index_time = _index_time + 1
			_index_target = _index_target + 1
			# _index_frame = _index_frame + 1
		return fx 

	def calcEDyn(self):
		fx = 0
		_index_target = 0
		for target in self.target_data.targets:
			# this index is to get time index for excluding the first and the last positions for acceleration calculation
			_index_time = 0
			_max_index_time = len(self.target_data.targets_frames[_index_target])
			for _index_frame in self.target_data.targets_frames[_index_target]:
				# exclusing , see above note...
				if _index_time == 0 or _index_time >= _max_index_time - 1:
					_index_time = _index_time + 1
					continue
				# (a,b) = past frame position
				# (c,d) = current frame position
				# (e,f) = next frame position
				# also remember cm is the units
				a = self.target_data.targets[_index_target][_index_time-1][0]*100
				b = self.target_data.targets[_index_target][_index_time-1][1]*100
				c = self.target_data.targets[_index_target][_index_time  ][0]*100
				d = self.target_data.targets[_index_target][_index_time  ][1]*100
				e = self.target_data.targets[_index_target][_index_time+1][0]*100
				f = self.target_data.targets[_index_target][_index_time+1][1]*100
				diffterm= a**2 + a*(2*e - 4*c) + b**2 + b*(2*f - 4*d) + 4*c**2  - 4*c*e + 4*d**2  - 4*d*f + e**2  + f**2
				fx = fx + diffterm
				_index_time = _index_time + 1
			_index_target = _index_target + 1
		return fx 

	def calcEExc(self):
		# also remember cm is the units
		def computeEExcWithIndex(i,j,self, this_frame_target, index_frame):
			# a = self.target_data.targets[this_frame_target[i]][index_time][0]*100
			# b = self.target_data.targets[this_frame_target[i]][index_time][1]*100
			# c = self.target_data.targets[this_frame_target[j]][index_time][0]*100
			# d = self.target_data.targets[this_frame_target[j]][index_time][1]*100
			obj_id_i = this_frame_target[i]
			obj_id_j = this_frame_target[j]
			index_time_i = self.target_data.targets_frames[obj_id_i].index(index_frame)
			index_time_j = self.target_data.targets_frames[obj_id_j].index(index_frame)
			a = self.target_data.targets[obj_id_i][index_time_i][0]*100
			b = self.target_data.targets[obj_id_i][index_time_i][1]*100
			c = self.target_data.targets[obj_id_j][index_time_j][0]*100
			d = self.target_data.targets[obj_id_j][index_time_j][1]*100
			# print obj_id_i, obj_id_j
			return CSIG / (a**2 + b**2 + c**2 + d**2 -2*a*c - 2*b*d)
		# for this one we do need to loop over the frames instead of targets
		fx = 0
		_index_frame = 0
		for frame in self.processed_data.people:
			_index_target = 0
			this_frame_target = []
			for target in self.target_data.targets:
				# check if the current target is there in the current frame
				# and if yes we add to the array
				if _index_frame in self.target_data.targets_frames[_index_target]:
					this_frame_target.append(_index_target)
				_index_target = _index_target + 1

			# we need a subfunction to calculate energy below (combinatorial strategy)
			# print this_frame_target
			for i in range(0,len(this_frame_target)-1):
				for j in range(i+1, len(this_frame_target)):
						# fx = fx + computeEExcWithIndex(i,j,self, this_frame_target, target_data.targets_frames[_index_target].index(_index_frame))
						fx = fx + computeEExcWithIndex(i,j,self, this_frame_target, _index_frame) #, target_data) #.targets_frames[_index_target].index(_index_frame))

			_index_frame = _index_frame + 1
		return fx

	def calcEPer(self):
		return 0

	def calcEReg(self):
		fx = len(self.target_data.targets)
		# get all trajectory length and compute sum of inverse of it
		traj_lengths = np.array([len(x) for x in self.target_data.targets])
		# MIU_EPER idea is from the irconttracking paper
		fx = fx + sum (1./traj_lengths)*MIU_EPER
		return fx

	def computeEnergy(self):
		self.e_obs = self.calcEObs()
		self.e_dyn = self.calcEDyn()
		self.e_exc = self.calcEExc()
		self.e_per = self.calcEPer()
		self.e_reg = self.calcEReg()
		print self.e_obs, self.e_dyn, self.e_exc, self.e_per, self.e_reg
		return self.e_obs + EDYN_WEIGHT*self.e_dyn + EEXC_WEIGHT*self.e_exc + EPER_WEIGHT*self.e_per + EREG_WEIGHT*self.e_reg

	def calcEObsPrime(self):
		return 0

	def calcEDynPrime(self):
		return 0

	def calcEExcPrime(self):
		return 0

	def calcEPerPrime(self):
		return 0

	def computeEnergyPrime(self):
		self.e_obs_prime = self.calcEObsPrime()
		self.e_dyn_prime = self.calcEDynPrime()
		self.e_exc_prime = self.calcEExcPrime()
		self.e_per_prime = self.calcEPerPrime()
		# no ERegPrime as local minimization only changes state of points and not changing (adding / removing) new/old points
		return self.e_obs_prime + EDYN_WEIGHT*self.e_dyn_prime + EEXC_WEIGHT*self.e_exc_prime + EPER_WEIGHT*self.e_per_prime

def plotPoints(data, target_data, subtitle, fig_index):
	# ax = plt.axes()
	ax = plt.gca()
	colors = cm.rainbow(np.linspace(0, 1, target_data.max_target_id))
	_index_target = 0
	for target in target_data.targets:
		ppl_x = [x[0] for x in target]
		ppl_y = [x[1] for x in target]
		# if _index_target != 0:
		if _index_target != 0 and _index_target != 1:
			plt.plot(ppl_x, ppl_y,color = g_colors[_index_target], label = "Other tracked target trajectory")
		else:
			# plt.plot(ppl_x, ppl_y,color = 'blue', label = "Tracked target trajectory \n (of interest)")
			if _index_target == 0:
				plt.plot(ppl_x, ppl_y,color = 'blue', label = "First Tracked target trajectory \n (of interest)")
			else:
				plt.plot(ppl_x, ppl_y,color = g_colors[_index_target], label = " Second Tracked target trajectory \n (of interest)")
		_index_target = _index_target + 1
	two_x = []
	two_y = []
	for twoleg in data.twoleg:
		two_x = two_x + [x[1] for x in twoleg]
		two_y = two_y + [x[2] for x in twoleg]
	one_x = []
	one_y = []
	for oneleg in data.oneleg:
		one_x = one_x + [x[1] for x in oneleg]
		one_y = one_y + [x[2] for x in oneleg]
	ax.scatter(two_x+one_x, two_y+one_y, color='grey', s=18, alpha=.5, label="Identified Leg Positions")
	# ax.xaxis.grid(True, which="major", linestyle='dotted')
	# ax.yaxis.grid(True, which="major", linestyle='dotted')


	ax.text(.45,-.075,subtitle, horizontalalignment='center', transform=ax.transAxes, fontsize=8)

	# ax.set_xlabel(subtitle, fontsize=8)  
	ax.set_xlabel(r'$x(m)$', fontsize=8)  
	ax.set_ylabel(r'$y(m)$', fontsize=8)  
	ax.xaxis.set_label_coords(1.15,-0.025)
	ax.yaxis.set_label_coords(-0.15,0.5)
	plt.setp(ax.get_xticklabels(), fontsize=10)
	plt.setp(ax.get_yticklabels(), fontsize=10)
	# plt.xlabel(r'\textsc{Sensor Y axis}')  
	# plt.ylabel(r'\textsc{Sensor X axis}')
	
	# ax.axis([0, 10, -10, 10])
	# ax.axis('equal')
	# ax.axis([0, 5, -5, 5])
	
	if fig_index == 1:
		plt.plot([1.54542264635-0.27, 1.9049684261-.27], [-5, 5], linestyle='--', color='red', label="Ground Truth")
		plt.plot([1.54542264635+0.24, 1.9049684261+.24], [-5, 5], linestyle='--', color='red')
		# plt.legend(bbox_to_anchor=(1.15, 1), loc=2, borderaxespad=0.,prop={'size':10})
	else:
		# plt.plot([1.54542264635, 1.9049684261], [-5, 5], linestyle='--', color='red', label="Ground Truth")
		plt.plot([1.54542264635-0.27, 1.9049684261-.27], [-5, 5], linestyle='--', color='red', label="Ground Truth")
		plt.plot([1.54542264635+0.24, 1.9049684261+.24], [-5, 5], linestyle='--', color='red')
		plt.legend(bbox_to_anchor=(-1.70, 1), loc=2, borderaxespad=0.,prop={'size':10})
	# ax.set_aspect('equal','datalim')
	# figure = plt.gcf()
	# plt.subplots_adjust(left = (5/25.4)/figure.xsize, bottom = (4/25.4)/figure.ysize, right = 1 - (1/25.4)/figure.xsize, top = 1 - (3/25.4)/figure.ysize)
	ax.set_aspect('equal','box')
	# ax('scaling')
	# ax.set_autoscale_on(False)
	plt.ylim(-4,4)
	plt.xlim(0,3.5)

	start, end = ax.get_xlim()
	ax.xaxis.set_ticks(np.arange(start, end, 1.0))
	ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('$%1.0f$'))
	# plt.gcf().tight_layout()

	# plt.plot([1.616998, 1.787815], [-3.009284, 1.741627])
	# plt.savefig('occluded.pdf')
	# plt.savefig('occluded.eps')
	# plt.savefig('occluded.svg')
	# pl.dump(plt.gcf(), file('fig1.pickle','w'))
	# plt.show()

def getProcessedAndTargetDataFromFile(filename):
	f_handle = open(filename,'r')
	processed_data =  readProcessedData(f_handle)
	target_data = gatherPeopleTracks(processed_data)
	return processed_data, target_data

# processed_data =  readProcessedData(f_handle)
# target_data = gatherPeopleTracks(processed_data)
# targets, targets_ids = gatherPeopleTracks(processed_data)
# print target_data.targets_modes

# processed_data, target_data = getProcessedAndTargetDataFromFile('processed_data_1paper.csv')
# plotPoints(processed_data, target_data)

plt.rc('text', usetex = True)
plt.rc('font', family='serif')

processed_data1, target_data1 = getProcessedAndTargetDataFromFile('processed_data_1paper.csv')
# # plt.subplot(2,2,1)
# # plt.axes([0.1,.55,0.38,0.4])
# plt.axes([0.01,.09,0.40,0.85])
# plotPoints(processed_data1, target_data1, "(a)", 1)

processed_data2, target_data2 = getProcessedAndTargetDataFromFile('processed_data_2paper.csv')
# plt.subplot(2,2,2)
# plt.axes([0.56,.55,0.38,0.4])
plt.axes([0.01,.09,0.40,0.85])
plotPoints(processed_data2, target_data2, "(a)", 1)

processed_data3, target_data3 = getProcessedAndTargetDataFromFile('processed_data_3paper.csv')
# # plt.subplot(2,2,3)
# plt.axes([0.60,.09,0.40,0.85])
# plotPoints(processed_data3, target_data3, "(b)", 2)

processed_data4, target_data4 = getProcessedAndTargetDataFromFile('processed_data_4paper.csv')
# plt.subplot(2,2,4)
# plt.axes([0.56,.09,0.38,0.4])
plt.axes([0.60,.09,0.40,0.85])
plotPoints(processed_data4, target_data4, "(b)", 2)

# plt.tight_layout(pad=2.0)
# plt.gcf().set_size_inches(3.5,6.5)
plt.gcf().set_size_inches(8.0,4.5)

filename_save = 'occluded2'
plt.savefig(filename_save + '.pdf')
plt.savefig(filename_save + '.eps')
plt.savefig(filename_save + '.svg')

# plt.show()


# print target_data.targets, target_data.targets_ids
# print targets, targets_ids

# cont_tracking = ContTracking(processed_data, target_data)
# print cont_tracking.computeEnergy(), cont_tracking.computeEnergyPrime()


