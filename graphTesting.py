import numpy as np
import pylab as pl
from matplotlib import collections  as mc
from matplotlib import pyplot as plt

lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]
c = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])

lc = mc.LineCollection(lines, colors=c, linewidths=2)
fig, ax = plt.subplots()

ax.scatter([1.5,2,2.5], [1.5,2,2.5], color='k')
ax.scatter(0, 0, color='g')
ax.scatter(3, 3, color='r')

ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)

plt.show()