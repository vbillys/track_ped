#!/usr/bin/env python
import itertools
from munkres import Munkres
from scipy.spatial.distance import mahalanobis
# import book_format
# import book_plots

import numpy as np
# from matplotlib.patches import Circle, Rectangle, Polygon, Arrow, FancyArrow
import pylab as plt
# import gh_internal as gh
import matplotlib.cm as cm
import matplotlib.animation as animation

f_handle = open('ped_data.csv','r')

class AnimatedScatter(object):
	def __init__(self, data):
		self.data = data
		self.fig, self.ax = plt.subplots()
		# self.stream = self.data_stream()
		self.stream = (x for x in self.data)
		self.ani = animation.FuncAnimation(self.fig, self.update, interval=100, init_func=self.setup_plot, blit=True, frames=len(data)-1, repeat=False)

	def setup_plot(self):
		x, y, c = next(self.stream)
		# self.scat = self.ax.scatter(x, y, c=c, animated=True, s=128)
		# self.scat = self.ax.scatter(x, y, c=c, animated=True, cmap=plt.cm.coolwarm, s=128)
		self.scat = self.ax.scatter(x, y, c=c, animated=True, cmap=plt.cm.PuOr, s=128)
		self.scat2 = self.ax.scatter(x, y, c=c, animated=True, cmap=plt.cm.coolwarm, s=256, marker='+', linewidth=2)
		self.ax.axis([0, 6, -5, 5])
		return self.scat, self.scat2

	def update(self, i):
		data = next(self.stream)
		# self.scat.set_offsets(data[:2, :])
		self.scat.set_offsets(np.column_stack((data[0], data[1])))
		self.scat2.set_offsets(np.column_stack((data[0], data[1])))
		# self.scat._sizes = 300 * abs(data[2])**1.5 + 100
		# print np.array(data[2])
		# self.scat.set_array(np.array(data[2]))
		self.scat.set_color(data[2])
		self.scat2.set_color(data[2])
		# self.scat.set_color(np.array(data[2]))
		# self.scat.set_facecolor(np.column_stack((data[2], data[2])))
		# self.scat.set_array(np.matrix(data[2]))
		return self.scat, self.scat2
		# return self.scat,

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
	ax.axis([0, 6, -5, 5])
	# print points_tracked
	# print colors
	# a = AnimatedScatter(points_tracked)
	# plt.show()

def animatePoints(data):
	maxlen = len(max(data,key=len))
	print 'maxlen',maxlen
	colors = cm.rainbow(np.linspace(0, 1, maxlen))
	# colors = np.linspace(.1, .88, maxlen)
	# colors = np.linspace(-0.02, .02, maxlen)
	# colors = np.random.random(maxlen)
	# colors = [0, 105, 7, 8]
	points_timed = []

	for frame in data:
		if len(frame)>0:
			points_timed.append([])
			# points_timed[-1].append([])
			# points_timed[-1].append([])
			# points_timed[-1].append([])
			xs = []
			ys = []
			cs = []
			_i = 0
			for _pp in frame:
				# points_timed[-1][0].append(_pp[0]) # = points_timed[-1][0] + _pp[0]
				# points_timed[-1][1].append(_pp[1]) # = points_timed[-1][1] + _pp[1]
				# points_timed[-1][2].append(colors[_i])
				xs.append(_pp[0]) # = points_timed[-1][0] + _pp[0]
				ys.append(_pp[1]) # = points_timed[-1][1] + _pp[1]
				cs.append(colors[_i])
				_i = _i + 1
			points_timed[-1] = (xs, ys, cs)

	# print points_timed

	a = AnimatedScatter(points_timed)
	plt.show()


def processMunkres(points):
	return []
	pass

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
	print point_
	# point = list(grouper(2, point_))
	point = zip(*(iter(point_),) * 2)
	print point
	points.append(point)
print points
points_processed = processMunkres(points)
plotPoints(points)
animatePoints(points)
# plotPoints(points_processed)
# animatePoints(points_processed)

# gh.plot_estimate_chart_3()
