#!/usr/bin/env python
import itertools
from collections import namedtuple

import math
import time

import numpy as np
import pylab as plt
import matplotlib.cm as cm
import matplotlib.animation as animation

# f_handle = open('processed_data_5.csv','r')
f_handle = open('processed_data_8sim.csv','r')

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
		for target in target_data.targets:
			# check if the current target is there in the current frame
			# if _index_frame in target_data.targets_frames[_index_target]:
			_index_time = 0
			for _index_frame in target_data.targets_frames[_index_target]:
				# print _index_target
				# _target_x = target[target_data.targets_frames[_index_target].index(_index_frame)][0]
				# _target_y = target[target_data.targets_frames[_index_target].index(_index_frame)][1]
				_target_x = target[_index_time][0]
				_target_y = target[_index_time][1]
				# get detections in current frame
				_detections_x = np.array([ x[1] for x in processed_data.twoleg[_index_frame]+processed_data.oneleg[_index_frame] ] )
				_detections_y = np.array([ x[2] for x in processed_data.twoleg[_index_frame]+processed_data.oneleg[_index_frame] ] )
				# get weights for confidences
				_detections_w = np.array([ x[0] for x in processed_data.twoleg[_index_frame]+processed_data.oneleg[_index_frame] ] )
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
		for target in target_data.targets:
			# this index is to get time index for excluding the first and the last positions for acceleration calculation
			_index_time = 0
			_max_index_time = len(target_data.targets_frames[_index_target])
			for _index_frame in target_data.targets_frames[_index_target]:
				# exclusing , see above note...
				if _index_time == 0 or _index_time >= _max_index_time - 1:
					_index_time = _index_time + 1
					continue
				# (a,b) = past frame position
				# (c,d) = current frame position
				# (e,f) = next frame position
				# also remember cm is the units
				a = target_data.targets[_index_target][_index_time-1][0]*100
				b = target_data.targets[_index_target][_index_time-1][1]*100
				c = target_data.targets[_index_target][_index_time  ][0]*100
				d = target_data.targets[_index_target][_index_time  ][1]*100
				e = target_data.targets[_index_target][_index_time+1][0]*100
				f = target_data.targets[_index_target][_index_time+1][1]*100
				diffterm= a**2 + a*(2*e - 4*c) + b**2 + b*(2*f - 4*d) + 4*c**2  - 4*c*e + 4*d**2  - 4*d*f + e**2  + f**2
				fx = fx + diffterm
				_index_time = _index_time + 1
			_index_target = _index_target + 1
		return fx 

	def calcEExc(self):
		# also remember cm is the units
		def computeEExcWithIndex(i,j,self, this_frame_target, index_time):
			a = self.target_data.targets[this_frame_target[i]][index_time][0]*100
			b = self.target_data.targets[this_frame_target[i]][index_time][1]*100
			c = self.target_data.targets[this_frame_target[j]][index_time][0]*100
			d = self.target_data.targets[this_frame_target[j]][index_time][1]*100
			return CSIG / (a**2 + b**2 + c**2 + d**2 -2*a*c - 2*b*d)
		# for this one we do need to loop over the frames instead of targets
		fx = 0
		_index_frame = 0
		for frame in processed_data.people:
			_index_target = 0
			this_frame_target = []
			for target in target_data.targets:
				# check if the current target is there in the current frame
				# and if yes we add to the array
				if _index_frame in target_data.targets_frames[_index_target]:
					this_frame_target.append(_index_target)
				# we need a subfunction to calculate energy below (combinatorial strategy)
				for i in range(0,len(this_frame_target)-1):
					for j in range(i+1, len(this_frame_target)):
							fx = fx + computeEExcWithIndex(i,j,self, this_frame_target, target_data.targets_frames[_index_target].index(_index_frame))

				_index_target = _index_target + 1
			_index_frame = _index_frame + 1
		return fx

	def calcEPer(self):
		return 0

	def calcEReg(self):
		fx = len(target_data.targets)
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

def plotPoints(data, target_data):
	ax = plt.axes()
	colors = cm.rainbow(np.linspace(0, 1, target_data.max_target_id))
	_index_target = 0
	for target in target_data.targets:
		ppl_x = [x[0] for x in target]
		ppl_y = [x[1] for x in target]
		plt.plot(ppl_x, ppl_y,color = colors[_index_target])
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
	ax.scatter(two_x+one_x, two_y+one_y, color='grey', s=48)
	ax.xaxis.grid(True, which="major", linestyle='dotted')
	ax.yaxis.grid(True, which="major", linestyle='dotted')
	ax.axis([0, 10, -10, 10])
	plt.show()

processed_data =  readProcessedData(f_handle)
target_data = gatherPeopleTracks(processed_data)
# targets, targets_ids = gatherPeopleTracks(processed_data)
# print target_data.targets_modes

# plotPoints(processed_data, target_data)

# print target_data.targets, target_data.targets_ids
# print targets, targets_ids

cont_tracking = ContTracking(processed_data, target_data)
print cont_tracking.computeEnergy(), cont_tracking.computeEnergyPrime()


