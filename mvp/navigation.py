import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt



def navigation_node(distance, angle):

    goal_distance = ctrl.Antecedent(np.arange(0, 100, 10), 'goal_distance')
    goal_direction = ctrl.Antecedent(np.arange(-90, 90, 10), 'goal_direction')

    angular_velocity = ctrl.Consequent(np.arange(-15, 16, 1), 'angular_velocity')
    linear_velocity = ctrl.Consequent(np.arange(0, 10, 1), 'linear_velocity')

    goal_direction.automf(5)
    goal_distance.automf(5)

    angular_velocity.automf(5)
    linear_velocity.automf(5)

    rule1 = ctrl.Rule(goal_distance['poor'], linear_velocity['poor'])
    rule2 = ctrl.Rule(goal_distance['good'], linear_velocity['good'])
    rule3 = ctrl.Rule(goal_distance['average'], linear_velocity['average'])
    rule4 = ctrl.Rule(goal_direction['poor'], angular_velocity['poor'])
    rule5 = ctrl.Rule(goal_direction['good'], angular_velocity['good'])
    rule6 = ctrl.Rule(goal_direction['average'], angular_velocity['average'])

    navigation_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])

    navigating = ctrl.ControlSystemSimulation(navigation_ctrl)

    navigating.input['goal_distance'] = distance
    navigating.input['goal_direction'] = angle

    #goal_direction.view()
    #plt.show()

    navigating.compute()

    #print(navigating.output['angular_velocity'])
    #print(navigating.output['linear_velocity'])

    return navigating.output['linear_velocity'], navigating.output['angular_velocity']



#print(navigation_node(1000, -15))