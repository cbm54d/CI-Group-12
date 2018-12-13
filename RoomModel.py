from typing import Callable, Tuple, List
from functools import reduce
from matplotlib import pyplot as plt
from matplotlib import collections as mc
import math
import numpy as np
from collections import deque
import FuzzyRules as FR

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
        def __init__(self, robotX=0, robotY=0, robotDir=0, goalX=1, goalY=1, goalThreshold=1):
                self.robotX = robotX
                self.robotY = robotY
                self.robotDir = robotDir        # Here, 0 refers to 0 degrees (what will be displayed as "up" or "north")
                self.goalX = goalX
                self.goalY = goalY
                self.goalThreshold = goalThreshold
                self.walls = []                         # This list will contain wall objects (containing two coordinate pairs representing the wall's endpoints)
                self.robotPathX = []            # This list will contain the x coordinates of the robot at every previous time interval (in order)
                self.robotPathY = []            # Same as above but for the y coordinates
                self.robotPathA = []            # Same as above but for the angle representing the direction the robot is facing at each step (in degrees)

        def updateRobot(self, deltaAngle, deltaDist):           # Updates robot based on values passed from back-end
                self.robotPathX.append(self.robotX)                                                     # Records the location and angle of the robot before moving it
                self.robotPathY.append(self.robotY)
                self.robotPathA.append(self.robotDir)
                self.robotDir = self.robotDir + deltaAngle                                                                              # Update direction robot is facing
                self.robotX = self.robotX + (deltaDist * math.sin(math.radians(self.robotDir))) # Using updated direction, update x-coordinate of robot
                self.robotY = self.robotY + (deltaDist * math.cos(math.radians(self.robotDir))) # Same as above for y-coordinate

        def goalCheck(self):            # Checks to see if the robot has reached the goal
                # This function will work by succeeding when the robot comes within a certain distance of the point representing the goal
                # AT ANY POINT during its movement from the last time step to the current time step. Not exactly sure how that'll work but here goes...
                # I'll START by having it identify if the robot ENDS UP within range of the goal at a given step
                goalInfo = self.computeGoalInfo()
                if goalInfo[0] < self.goalThreshold:
                        return True
                else:
                        return False

        def displayRoom(self):          # Creates a graphical display of the room's state at the time it is called
                # I found this method for graphing multiple line segments here:
                # https://stackoverflow.com/questions/21352580/matplotlib-plotting-numerous-disconnected-line-segments-with-different-colors
                lines = []
                for wall in self.walls:
                        lines.append(wall.all)
                #print lines
                # https://stackoverflow.com/questions/36470343/how-to-draw-a-line-with-matplotlib/36479941


                x = [self.robotX, self.robotX+0.2*math.sin(math.radians(self.robotDir))]
                y = [self.robotY, self.robotY+0.2*math.cos(math.radians(self.robotDir))]
                c = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
                lc = mc.LineCollection(lines, linewidths=2)
                goalZone = plt.Circle((self.goalX, self.goalY), self.goalThreshold, color='g')
                fig, ax = plt.subplots()
                ax.add_artist(goalZone)
                ax.plot(x, y)
                ax.add_collection(lc)
                
                ax.scatter(self.robotPathX, self.robotPathY, color='grey')
                ax.scatter(self.robotX, self.robotY, color='blue')
                ax.scatter(self.goalX, self.goalY, color='green')

                #ax.autoscale()
                ax.margins(0.1)
                ax.axis('scaled')

                plt.show()


        # Okay, let's take a moment to think about how this thing should work GIVEN THAT SOME LINES CAN BE VERTICAL.
        # Vertical lines have an UNDEFINED slope (involves a division by zero), so they cause problems for the program.
        # Still, if I can properly identify vertical lines/slopes early on, then I can use that property to get the 
        # information I need without doing any of the computations that could potentially involve a program-ending 
        # division by zero. So, what are all the cases which may cause this problem?
        #       CASE 1s: a wall with a vertical slope
        #               - identifiable by:                              wall.x1 == wall.x2
        #               - mathematical implications:    yBound1 == wall.y1 == yBound2 == wall.y2
        #               This case is extra tricky because xBounds WON'T WORK. Still, it can easily be determined whether or not the robot 
        #               is perpendicular to the wall (True) by using the following expression:
        #                               ((wall.y1 >= self.robotY and wall.y2 <= self.robotY) and (wall.y1 <= self.robotY and wall.y2 >= self.robotY))
        #       CASE 2s: a wall with a horizontal slope -> (this causes the perpendicular slope used in calculations to be vertical (_/0))
        #               - identifiable by:                              wall.y1 == wall.y2
        #               - mathematical implications:    xBound1 == xBound2 == wall.x1 == wall.x2
        #       CASE 3s: vertical slope (for distance vector) resulting from the robot being directly above/below the nearest endpoint of a wall
        #               - identifiable by:                              wall.x1 == self.robotX or wall.x2 == self.robotX
        #               - mathematical implications:    angle == 180 or 0 depending on if robot is above or below endpoint in question
        #               This doesn't screw up the parts of the code that have to do with distance since, in the case where the robot is not on
        #               a perpendicular with the wall, the pythagorean theorem is all that is required to calculate the distance. However, when
        #               computing the ANGLE of the nearest point to the robot, a tan(_/0) is used, so this case must still be accounted for.

        # And now the fun REALLY begins...
        def computeObjInfo(self):

                minDist = 1000
                minAngle = 0

                for wall in self.walls:

                        dist = None
                        angle = None
                        perpSlope = None
                        slope = None
                        robotPerpToWall = False

                        # Account for cases which would otherwise involve dividing by zero
                        # CASE 1s: wall with a vertical (undefined) slope
                        if wall.x1 == wall.x2:
                                #print "~~! CASE 1s: wall with vertical slope !~~"
                                if self.robotY >= wall.y1 and self.robotY <= wall.y2:
                                        robotPerpToWall = True
                                if self.robotY <= wall.y1 and self.robotY >= wall.y2:
                                        robotPerpToWall = True

                        # CASE 2s: wall with a horizontal slope & vertical (undefined) distance vector
                        elif wall.y1 == wall.y2:
                                #print "~~! CASE 2s: wall with horizontal slope !~~"
                                if self.robotX >= wall.x1 and self.robotX <= wall.x2:
                                        robotPerpToWall = True
                                if self.robotX <= wall.x1 and self.robotX >= wall.x2:
                                        robotPerpToWall = True

                        else:
                        # Determine if robot is on line perpendicular to wall being assessed
                                slope = (wall.y2 - wall.y1) / (wall.x2 - wall.x1)
                                yIntercept = wall.y1 - (slope * wall.x1)
                                perpSlope = -((wall.x2 - wall.x1) / (wall.y2 - wall.y1))
                                perpYIntercept1 = wall.y1 - (perpSlope * wall.x1)
                                xBound1 = (self.robotY - perpYIntercept1) / perpSlope
                                perpYIntercept2 = wall.y2 - (perpSlope * wall.x2)
                                xBound2 = (self.robotY - perpYIntercept2) / perpSlope
                                #print("xBound1 = {0}       xBound2 = {1}".format(xBound1, xBound2))
                                if self.robotX >= xBound1 and self.robotX <= xBound2:
                                        robotPerpToWall = True
                                elif self.robotX <= xBound1 and self.robotX >= xBound2:
                                        robotPerpToWall = True

                        # Calculate distance from robot to nearest point along wall
                        if robotPerpToWall:                                                                                     # Case 1: robot is on a perpendicular with the wall
                                #print("Wall is perpendicular to the robot")                            # CASE 1s: wall with a vertical (undefined) slope
                                if wall.x1 == wall.x2: 
                                        dist = abs(self.robotX - wall.x1)
                                        angleFromZero = 90 if self.robotX > wall.x1 else 270

                                elif wall.y1 == wall.y2:                                                                # CASE 2s: wall with a horizontal slope
                                        dist = abs(self.robotY - wall.y1)
                                        angleFromZero = 180 if self.robotY > wall.y1 else 0

                                else:
                                        perpYInterceptR = self.robotY - (perpSlope * self.robotX)
                                        nearestPoint = line_intersect(wall.p1, (self.robotX, self.robotY), slope, perpSlope)
                                        dist = math.sqrt((self.robotX - nearestPoint[0]) ** 2 + (self.robotY - nearestPoint[1]) ** 2)
                                        #print("angle = acos({0})".format(abs(self.robotY - nearestPoint[1])/dist))
                                        angleFromZero = math.degrees(math.acos((nearestPoint[1] - self.robotY)/dist))

                        else:                                                                                                           # Case 2: robot is not on perpendicular with the wall
                                #print("Wall is not perpendicular to the robot")
                                dist1 = math.sqrt((self.robotX - wall.x1) ** 2 + (self.robotY - wall.y1) ** 2)
                                dist2 = math.sqrt((self.robotX - wall.x2) ** 2 + (self.robotY - wall.y2) ** 2)
                                dist = min([dist1, dist2])
                                if dist == dist1:
                                        p = (wall.x1, wall.y1)
                                else:
                                        p = (wall.x2, wall.y2)

                                if wall.x1 == self.robotX or wall.x2 == self.robotX:    # CASE 3s: vertical distance vector to wall endpoint
                                        angleFromZero = 180 if self.robotY > wall.y1 else 0
                                else:
                                        angleFromZero = math.degrees(math.atan((self.robotX - p[0]) / (self.robotY - p[1])))

                        angle = angleFromZero - self.robotDir
                        #print( "for some wall: dist = {0} & angle = {1}".format(dist, angle))
                        if dist < minDist:
                                minDist = dist
                                minAngle = angle

                return (minDist, minAngle)

        def computeGoalInfo(self):
                dx = self.goalX - self.robotX
                dy = self.goalY - self.robotY
                goalDist = math.sqrt(dx**2 + dy**2)
                goalAngle = math.degrees(math.atan(dy/dx))
                return (goalDist, goalAngle)

# The function below comes from https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
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

        a = np.array([lhs0, lhs1])
        b = np.array([rhs0, rhs1])
        try:
                px = np.linalg.solve(a, b)
        except:
                px = np.array([np.nan, np.nan])

        return px

def initialize():
        print("Initializing StateModel object...")
        # StateModel(robotX, robotY, robotAngle, goalX, goalY, goalThreshold)
        stateModel = StateModel(1, 1, 45, 4, 4, 0.25)
        print("Generating walls...")
        # The walls listed below define the boundaries of the room, which is currently expressed as a 5x5 grid.
        boundaries = [Wall(0,0,5,0), Wall(5,0,5,5), Wall(5,5,0,5), Wall(0,5,0,0)]
        for wall in boundaries:
                stateModel.walls.append(wall)

        # For the time being, extra walls must be hard-coded here.
        # To add a wall, include Wall(x1, x2, y1, y2) in the list assigned to the walls variable.
        walls = [Wall(2,3,3,2)]
                   #[Wall(2,3,3,2), Wall(2,1,3,2), Wall(4,2,5,3), Wall(3,3,3,4)]
                   #[Wall(1,2,2,3), Wall(3,2,3,3), Wall(3,3,5,3)]
        for wall in walls:
                stateModel.walls.append(wall)

        return stateModel

def main():

        # A lot of the below code is designed for testing purposes.
        # The contents of main() should be manipulated to suit the needs of the one utilizing the program at any given time.

        print("Now starting main...")
        stateModel = initialize()
        stateModel.displayRoom()
        #(dist, angle) = stateModel.computeObjInfo()
        #print("from main:\ndist = {0}\nangle = {1}".format(dist, angle))
        #moveList = [
        #               [0,0.7],   [0,0.5],   [-30,0.45],
        #               [-21,0.4], [-5,0.3],  [19,0.2],
        #               [35,0.5],  [30,0.57], [4,0.7], 
        #               [0,0.4]]

        #goalReached = False
        #noMoreMoves = False
        #moveQueue = deque(moveList)
        #while not goalReached and not noMoreMoves:
        #       move = moveQueue.popleft()
        #       stateModel.updateRobot(move[0], move[1])
        #       goalReached = stateModel.goalCheck()
        #       noMoreMoves = len(moveQueue) == 0

#       stateModel.displayRoom()
#       if goalReached:
#               print "GOAL REACHED!!!"
#       elif noMoreMoves:
#               print "The test has run out of moves without reaching the goal."
#       else:
#               print "I don't think this should happen..."

        while not stateModel.goalCheck():
            (oD, oA) = stateModel.computeObjInfo()
            (gD, gA) = stateModel.computeGoalInfo()
            output = FR.rulebook.doFuzzyLogic(objectDistance = oD,
                                              objectAngle = oA,
                                              goalDistance = gD,
                                              goalAngle = gA)
            print('Object Angle        : {}'.format(oA))
            print('Object Distance     : {}'.format(oD))
            print('Goal Angle          : {}'.format(gA))
            print('Goal Distance       : {}'.format(gD))
            print('Calculated direction: {}'.format(output['direction']))
            print('Calculated speed    : {}'.format(output['speed']))
            print('Calculated direction: {}\n'.format(output['direction']))
            stateModel.updateRobot(output['direction'], output['speed'] * 0.1)
            stateModel.displayRoom()
main()
