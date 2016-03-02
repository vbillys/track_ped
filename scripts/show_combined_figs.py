
# import pylab as plt
# import pickle as pl
# import numpy as np
import math

import numpy as np
import pylab as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import matplotlib.ticker as ticker

import pickle as pl
import brewer2mpl


bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
g_colors = bmap.mpl_colors

# fig1_handle = pl.load(open('fig1.pickle','rb'))
# fig1_handle.show(True)
f_handle = open('ped_gather_3paper.csv','r')


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
	point = zip(*(iter(point_),) * 2)
	# print point
	points.append(point)

# print points
angles = []
ranges = []
for point in points:
	angles.append(math.atan(point[0][1]/point[0][0]))
	ranges.append(math.hypot(point[0][0],point[0][1]))


angles = angles[0:1050]
ranges = ranges[0:1050]

plt.rc('text', usetex = True)
plt.rc('font', family='serif')

fig, ax1 = plt.subplots()

t = np.arange(0.1, 0.1+0.1*len(angles), 0.1)
ax1.plot(t, angles, color=g_colors[0], label =r'$\theta_p$')
# plt.plot(angles)
# plt.plot(ranges)
# plt.xlabel('Running time(s)')
ax1.set_xlabel('time (s)')
ax1.set_ylabel(r'angle $\theta_P$(rad)')#, color=g_colors[0])
plt.ylim(-3,3)
plt.xlim(0,0.1*len(angles))
plt.legend(bbox_to_anchor=(0.15, 0.2), loc=2, borderaxespad=0.,prop={'size':10})
# plt.legend()

ax2 = ax1.twinx()
ax2.plot(t, ranges, color=g_colors[1], label ='$R_p$')
ax2.set_ylabel('range $R_p$(m)')#, color=g_colors[1])
plt.ylim(-3,3)
plt.xlim(0,0.1*len(angles))

plt.legend(bbox_to_anchor=(0.15, .27), loc=2, borderaxespad=0.,prop={'size':10})
# plt.legend()
# plt.ylabel()
plt.savefig('pursuit_control.pdf')
plt.show()
