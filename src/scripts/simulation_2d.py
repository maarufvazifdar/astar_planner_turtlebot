#!/usr/bin/env python
import time
import argparse
import astar


if __name__ == '__main__':
    # Getting data from the user
    input_parser = argparse.ArgumentParser()
    input_parser.add_argument('--start_location', default="[1, 1, 90]")
    input_parser.add_argument('--goal_location', default="[6, 2, 0]")
    input_parser.add_argument('--robot_radius', default=0.177)
    input_parser.add_argument('--clearance', default=0.1)
    input_parser.add_argument('--theta_step', default=30)
    input_parser.add_argument('--step_size', default=2)
    input_parser.add_argument('--goal_threshold', default=0.1)
    input_parser.add_argument('--wheel_radius', default=0.028)
    input_parser.add_argument('--wheel_length', default=0.320)
    input_parser.add_argument('--RPM', default="[3,3]")
    input_parser.add_argument('--weight', default=1)
    input_parser.add_argument('--displayExploration', default=0)
    input_parser.add_argument('--displayPath', default=1)
    input_parser.add_argument('--thresh', default=0.01)
    args = input_parser.parse_args()

    # Defining argument data to variables
    start_location = args.start_location
    goal_location = args.goal_location
    step_size = int(args.step_size)
    thresh = float(args.thresh)
    goal_threshold = float(args.goal_threshold)
    r = float(args.robot_radius)
    c = float(args.clearance)

    # Start location [x, y, theta]
    source_location = [float(i) for i in start_location[1:-1].split(',')]
    # Goal location [x, y]
    goal_location = [float(i) for i in goal_location[1:-1].split(',')]

    print("Source location:", source_location)
    print("Goal location:", goal_location)

    # Differential-drive constraints
    rpm = [float(i) for i in args.RPM[1:-1].split(',')]
    Ur, Ul = rpm[0], rpm[1]
    wheel_length = float(args.wheel_length)
    wheel_radius = float(args.wheel_radius)
    weight = float(args.weight)

    path_planner = astar.Astar(
        source_location, goal_location, step_size=step_size,
        goal_threshold=goal_threshold, width=10, height=10, thresh=thresh, r=r,
        c=c, wheel_length=wheel_length, Ur=Ur, Ul=Ul,
        wheel_radius=wheel_radius, weight=weight,
        displayExploration=int(args.displayExploration),
        displayPath=int(args.displayPath))

    path_planner.findOptimalPath()

    print("Optimal path\n", path_planner.trace_index)

    # Output parameters
    for idx in path_planner.trace_index:
        ul, ur = path_planner.actions[int(idx)]
        dx = wheel_radius * 0.5 * (ur + ul)
        dt = wheel_radius * (ur - ul) / wheel_length
        print(wheel_radius, wheel_length, ul, ur, dx, dt)
