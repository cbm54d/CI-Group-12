import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

def avoidance_node(distance, angle):
    obstacle_direction = ctrl.Antecedent(np.arange(-90, 90, 10), 'obstacle_direction')
    obstacle_distance = ctrl.Antecedent(np.arange(0,100,1), 'obstacle_distance')

    angluar_velocity = ctrl.Consequent(np.arange(-15, 16, 1), 'angular_velocity')

    obstacle_direction['left'] = fuzz.trimf(obstacle_direction.universe, [5, 90, 90])
    obstacle_direction['right'] = fuzz.trimf(obstacle_direction.universe, [-90, -90, -5])
    obstacle_direction['center'] = fuzz.trimf(obstacle_direction.universe, [-10, 0, 10])
    obstacle_distance.automf(5)

    angluar_velocity.automf(5)

    rule1 = ctrl.Rule(obstacle_direction["right"] & obstacle_distance["poor"], angluar_velocity["good"])
    rule2 = ctrl.Rule(obstacle_direction["center"] & obstacle_distance["poor"], angluar_velocity["good"])
    rule3 = ctrl.Rule(obstacle_direction["center"] & obstacle_distance["average"], angluar_velocity["good"])
    rule4 = ctrl.Rule(obstacle_direction["left"] & obstacle_distance["poor"], angluar_velocity["poor"])
    rule5 = ctrl.Rule(obstacle_distance["good"] | obstacle_distance["average"], angluar_velocity["average"])


    obstacle_avoidance_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])

    avoiding = ctrl.ControlSystemSimulation(obstacle_avoidance_ctrl)

    avoiding.input['obstacle_direction'] = angle
    avoiding.input['obstacle_distance'] = distance


    avoiding.compute()


    return avoiding.output['angular_velocity']




