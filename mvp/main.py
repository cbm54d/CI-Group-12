import turtle as t
import time
import numpy as np

from sys import platform as sys_pf
if sys_pf == 'darwin':
    import matplotlib
    matplotlib.use("TkAgg")

import robot_controller
import transformation





class Robot():
    def __init__(self, x_pos=0, y_pos=0, orientation=0):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.orientation = orientation

    def draw(self):

        t.hideturtle()
        t.speed(0)
        t.shapesize(0.5)
        t.penup()
        t.goto(self.x_pos, self.y_pos)
        t.shape('square')
        t.tiltangle(self.orientation)
        t.stamp()


class CircularObstacle():
    def __init__(self, center_x=0, center_y=0, radius=10):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius

    def draw(self):
        t.hideturtle()
        t.speed(0)


        t.penup()
        t.goto(self.center_x, self.center_y)
        t.dot(2*self.radius, 'red')
        t.stamp()

    def distanceTo(self, x, y):
        return np.sqrt(pow(x - self.center_x, 2) + pow(y - self.center_y, 2)) - self.radius


robot = Robot()

goal_x = 300
goal_y = 300
t.goto(goal_x,goal_y)
t.shape('triangle')
t.stamp()

o1 = CircularObstacle(200, 150, 50)
o2 = CircularObstacle(380, 250, 30)

obstacles = [o1, o2]

for o in obstacles:
    o.draw()



while True:


    goal_distance = np.sqrt(pow(goal_x - robot.x_pos, 2) + pow(goal_y - robot.y_pos, 2))

    if goal_distance < 5:
        break

    goalOrientation = transformation.getGoalOrientation(goal_x, goal_y, robot.x_pos, robot.y_pos)
    wHr = transformation.getTransformationMatrix(robot.orientation, robot.x_pos, robot.y_pos)
    wHg = transformation.getTransformationMatrix(goalOrientation, goal_x, goal_y)

    gHr = wHg.I * wHr
    rHg = gHr.I

    goal_angle = transformation.getRotationAngle(rHg)

    closest_obstacle = obstacles[0]

    for o in obstacles:
        if o.distanceTo(robot.x_pos, robot.y_pos) < closest_obstacle.distanceTo(robot.x_pos, robot.y_pos):
            closest_obstacle = o

    obstacleOrientation = transformation.getGoalOrientation(closest_obstacle.center_x, closest_obstacle.center_y, robot.x_pos, robot.y_pos)
    wHo = transformation.getTransformationMatrix(obstacleOrientation, closest_obstacle.center_x, closest_obstacle.center_y)
    oHr = wHo.I * wHr
    rHo = oHr.I
    obstacle_angle = transformation.getRotationAngle(rHo)
    obstacle_distance = closest_obstacle.distanceTo(robot.x_pos, robot.y_pos)

    v, w = robot_controller.robot_controller(goal_distance, goal_angle, obstacle_distance, obstacle_angle)


    robot.x_pos = robot.x_pos + v * np.cos(robot.orientation / 180 * np.pi)
    robot.y_pos = robot.y_pos + v * np.sin(robot.orientation / 180 * np.pi)
    robot.orientation = robot.orientation + w

    robot.draw()



    time.sleep(0.1)


t.done()







