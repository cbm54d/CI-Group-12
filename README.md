# CI Project Group 12

## RoomModel.py
This file is responsible for doing all of the front-end work in this project. By communicating with the (fuzzy) back-end, this file provides the means to keep track of the state of the simulation, compute the values picked up from the robot's "sensors" (angle and distance of nearest part of nearest object & angle and distance of the goal), update the model, and display all pertinent information about the model accordingly.

Currently, to edit the simulation settings, one must open RoomModel.py and change the hard-coded values used in the initialize() function. Also, if one wishes to provide a predetermined set of commands for the robot to follow (in the form of a list of 2-element lists of the form [desired_change_in_angle, desired_distance_to_travel]) a sequence of commands can be entered into the moveList variable inside the main() function before running the program.



## MVP version

There is a MVP (Minimum Viable Product) version code at branch `mvp`, with simplest implementation. Just run `main.py` inside `mvp` folder.