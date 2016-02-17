#!/usr/bin/env python
import itertools
from collections import namedtuple

import math
import time

import numpy as np
import pylab as plt
import matplotlib.cm as cm
import matplotlib.animation as animation

f_handle = open('processed_data_5.csv','r')
# f_handle = open('processed_data_8sim.csv','r')

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
TargetData = namedlist('TargetData',('targets', 'targets_ids', 'max_target_id'))

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
				ppl_ = zip(*(iter(point_),) * 4)
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

	target_data = TargetData([[] for _ in range(max_obj_id)], [[] for _ in range(max_obj_id)], max_obj_id)
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
			target_data.targets_ids[int(target[1])-1].append(_index_frame)
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
plotPoints(processed_data, target_data)
# print target_data.targets, target_data.targets_ids
# print targets, targets_ids


