import obstacle_avoidance
import navigation

def robot_controller(goal_distance, goal_angle, obstacle_distance, obstacle_angle):
    obstacle_avoidance_output = obstacle_avoidance.avoidance_node(obstacle_distance, obstacle_angle)

    if (abs(obstacle_avoidance_output) < 1):
        return navigation.navigation_node(goal_distance, goal_angle)
    else:
        return 5, obstacle_avoidance_output