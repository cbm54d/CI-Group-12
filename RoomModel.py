from typing import Callable, Tuple, List
from functools import reduce
from matplotlib import pyplot as plt
from matplotlib import collections as mc
import math
import numpy as np
from scipy.optimize import fsolve
#import pylab as pl

class Wall:
	def __init__(self, x1, y1, x2, y2):
		self.x1 = x1
		self.x2 = x2
		self.y1 = y1
		self.y2 = y2
		self.p1 = (x1, y1)
		self.p2 = (x2, y2)
		self.all = (self.p1, self.p2)

class StateModel:
	def __init__(self, robotX=0, robotY=0, robotDir=0, goalX=1, goalY=1):
		self.robotX = robotX
		self.robotY = robotY
		self.robotDir = robotDir	# Here, 0 refers to 0 degrees (what will be displayed as "up" or "north")
		self.goalX = goalX
		self.goalY = goalY
		self.walls = []				# This list will contain wall objects (containing two coordinate pairs representing the wall's endpoints)
		self.robotPathX = [0.1, 0.4, 0.7]		# This list will contain the x coordinates of the robot at every previous time interval (in order)
		self.robotPathY = [0.1, 0.4, 0.7]		# Same as above but for the y coordinates

	def updateRobot(self, deltaAngle, deltaDist):		# Updates robot based on values passed from back-end
		self.robotDir = self.robotDir + deltaAngle					# Update direction robot is facing
		self.robotX = self.robotX + math.cos(90 - self.robotDir)	# Using updated direction, update x-coordinate of robot
		self.robotY = self.robotY + math.sin(90 - self.robotDir)	# Same as above for y-coordinate

	def displayRoom(self):		# Creates a graphical display of the room's state at the time it is called
		# I found this method for graphing multiple line segments here:
		# https://stackoverflow.com/questions/21352580/matplotlib-plotting-numerous-disconnected-line-segments-with-different-colors
		#lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]
		lines = []
		for wall in self.walls:
			lines.append(wall.all)
		print lines
		c = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
		# Later, replace hard-coded "lines" with self.walls *****
		lc = mc.LineCollection(lines, linewidths=2)
		fig, ax = plt.subplots()
		ax.add_collection(lc)

		ax.scatter(self.robotPathX, self.robotPathY, color='k')
		ax.scatter(self.robotX, self.robotY, color='g')
		ax.scatter(self.goalX, self.goalY, color='r')

		ax.autoscale()
		ax.margins(0.1)

		plt.show()

	# And now the fun REALLY begins...
	def computeObjInfo(self):

		minDist = 1000
		minAngle = 0

		for wall in self.walls:

			dist = None
			angle = None

			# TODO: Account for if the line is vertical, making the slope 1/0 *****
			# Determine if robot is on line perpendicular to wall being assessed
			robotPerpToWall = False
			slope = (wall.y2 - wall.y1) / (wall.x2 - wall.x1)
			yIntercept = wall.y1 - (slope * wall.x1)
			perpSlope = -((wall.x2 - wall.x1) / (wall.y2 - wall.y1))
			perpYIntercept1 = wall.y1 - (perpSlope * wall.x1)
			xBound1 = (self.robotY - perpYIntercept1) / perpSlope
			perpYIntercept2 = wall.y2 - (perpSlope * wall.x2)
			xBound2 = (self.robotY - perpYIntercept2) / perpSlope
			print("xBound1 = {0}       xBound2 = {1}".format(xBound1, xBound2))
			if self.robotX >= xBound1 and self.robotX <= xBound2:
				robotPerpToWall = True
			elif self.robotX <= xBound1 and self.robotX >= xBound2:
				robotPerpToWall = True

			# Case 1: robot is on a perpendicular with the wall
			if robotPerpToWall:	# First, find point along line nearest the robot (using the perpendicular slope)
				print("Wall is perpendicular to the robot")
				perpYInterceptR = self.robotY - (perpSlope * self.robotX)
				print("perpSlope = {0}".format(perpSlope))
				print("slope = {0}".format(slope))
				print("perpYInterceptR = {0}".format(perpYInterceptR))
				print("yIntercept = {0}".format(yIntercept))

				#nearestPoint = get_intersect(wall.p1, wall.p2, (self.robotX, self.robotY), (0, perpYInterceptR))

				nearestPoint = line_intersect(wall.p1, (self.robotX, self.robotY), slope, perpSlope)

				# a = np.array([[-perpSlope, 1], [slope, 1]])
				# b = np.array([perpYInterceptR, yIntercept])
				# nearestPoint = np.linalg.solve(a, b)			# https://docs.scipy.org/doc/numpy-1.13.0/reference/generated/numpy.linalg.solve.html
				dist = math.sqrt((self.robotX - nearestPoint[0]) ** 2 + (self.robotY - nearestPoint[1]) ** 2)
				angle = math.tan(perpSlope)

			else:
				print("Wall is not perpendicular to the robot")
				dist1 = math.sqrt((self.robotX - wall.x1) ** 2 + (self.robotY - wall.y1) ** 2)
				dist2 = math.sqrt((self.robotX - wall.x2) ** 2 + (self.robotY - wall.y2) ** 2)
				dist = min([dist1, dist2])
				if dist == dist1:
					p = (wall.x1, wall.y1)
				else:
					p = (wall.x2, wall.y2)
				angle = math.tan((self.robotY - p[1]) / (self.robotX - p[0]))

			print( "for some wall: dist = {0} & angle = {1}".format(dist, angle))
			if dist < minDist:
				print("since {0} < {1}, new minDist is {0}".format(dist, minDist))
				minDist = dist
				minAngle = angle

		return (minDist, minAngle)

def get_intersect(a1, a2, b1, b2):
	""" 
	Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
	a1: [x, y] a point on the first line
	a2: [x, y] another point on the first line
	b1: [x, y] a point on the second line
	b2: [x, y] another point on the second line
	"""
	s = np.vstack([a1,a2,b1,b2])        # s for stacked
	h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
	l1 = np.cross(s[0], s[1])           # get first line
	l2 = np.cross(s[2], s[3])           # get second line
	x, y, z = np.cross(l1, l2)          # point of intersection
	if z == 0:                          # lines are parallel
			return (float('inf'), float('inf'))
	return (x/z, y/z)

def line_intersect(p0, p1, m0=None, m1=None, q0=None, q1=None):
	''' intersect 2 lines given 2 points and (either associated slopes or one extra point)
	Inputs:
		p0 - first point of first line [x,y]
		p1 - fist point of second line [x,y]
		m0 - slope of first line
		m1 - slope of second line
		q0 - second point of first line [x,y]
		q1 - second point of second line [x,y]
	'''
	if m0 is  None:
		if q0 is None:
			raise ValueError('either m0 or q0 is needed')
		dy = q0[1] - p0[1]
		dx = q0[0] - p0[0]
		lhs0 = [-dy, dx]
		rhs0 = p0[1] * dx - dy * p0[0]
	else:
		lhs0 = [-m0, 1]
		rhs0 = p0[1] - m0 * p0[0]

	if m1 is  None:
		if q1 is None:
			raise ValueError('either m1 or q1 is needed')
		dy = q1[1] - p1[1]
		dx = q1[0] - p1[0]
		lhs1 = [-dy, dx]
		rhs1 = p1[1] * dx - dy * p1[0]
	else:
		lhs1 = [-m1, 1]
		rhs1 = p1[1] - m1 * p1[0]

	a = np.array([lhs0, 
					lhs1])

	b = np.array([rhs0, 
					rhs1])
	try:
		px = np.linalg.solve(a, b)
	except:
		px = np.array([np.nan, np.nan])

	return px

# FOR EACH wall:
#   IF robot is at a location s.t. it lies along a line perpendicular to a wall:
#     THEN find the point along the wall that is closest to the robot (using appropriate equation)
#     ELSE find the distances between the robot and each endpoint of the wall (using pythagorean theorem)
#       IF endpoint 1 is closer, note that it is the closest point to the robot along that wall
#       ELSE note that endpoint 2 is the closest point to the robot along that wall
#       IF shortest distance to line is less than previous minimum distance to a wall
#         THEN set new distance as minimum distance to any wall
#           (& record point along the wall as the nearest point to the robot - necessary for computing angle.)
# (Once every wall has been analyzed and the nearest point has been found...)
# Compute the angle to nearest object based on the point found in the previous section

def main():
	print("Now starting main...")
	print("Initializing StateModel object...")
	stateModel = StateModel(1, 1, 45, 4, 4)
	stateModel.displayRoom
	print("Generating walls...")
	#walls = [[(2,3),(3,2)], [(4,2),(5,3)]]
	walls = [Wall(2,3,3,2), Wall(4,2,5,3)]
	for wall in walls:
		stateModel.walls.append(wall)
	(dist, angle) = stateModel.computeObjInfo()
	print("from main:\ndist = {0}\nangle = {1}".format(dist, angle))
	stateModel.displayRoom()

main()





# robotX=0, robotY=0, robotDir=0, goalX=1, goalY=1







		

