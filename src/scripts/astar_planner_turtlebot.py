#!/usr/bin/env python3
import rospy
import argparse
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from simulation_2d import *


def driveTurtlebot(args, path_planner, velocities=None):
    r = float(args.wheel_radius)
    l = float(args.wheel_length)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd = Twist()

    rate = rospy.Rate(10.0)
    rospy.sleep(5.0)
    rospy.loginfo("Moving Turtlebot ...")
    rospy.loginfo(len(path_planner.trace_index))

    for i in path_planner.trace_index:
        ul, ur = path_planner.actions[int(i)]
        dx = r * 0.5 * (ur + ul)
        dt = r * (ur - ul) / l
        start_location = time.time()
        while(time.time() - start_location <= 1):
            cmd.linear.x = dx
            cmd.angular.z = dt
            pub.publish(cmd)
            print(r, l, ul, ur, dx, dt)

    cmd.linear.x = 0
    cmd.angular.z = 0
    pub.publish(cmd)
    rospy.loginfo("Goal Reached !!!")


def startPose(start_location):
    print("Setting robot start pose")
    rospy.sleep(5.0)

    x = start_location[0]
    y = start_location[1]
    a = start_location[2] * 3.14 / 180.0

    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_burger'
    state_msg.reference_frame = 'world'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = np.sin(a / 2)
    state_msg.pose.orientation.w = np.cos(a / 2)
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException:
        print("Service failed:")


def main():
    rospy.init_node('set_pose')
    Parser = argparse.ArgumentParser()
    Parser.add_argument("--start_location", default="[1,1,90]")
    Parser.add_argument('--goal_location', default="[6, 2, 0]")
    Parser.add_argument('--robot_radius', default=0.177)
    Parser.add_argument('--clearance', default=0.1)
    Parser.add_argument('--displayExploration', default=0)
    Parser.add_argument('--displayPath', default=1)
    Parser.add_argument('--theta_step', default=30)
    Parser.add_argument('--step_size', default=2)
    Parser.add_argument('--thresh', default=0.01)
    Parser.add_argument('--goal_threshold', default=0.1)
    Parser.add_argument('--wheel_radius', default=0.028)
    Parser.add_argument('--wheel_length', default=0.320)
    Parser.add_argument('--RPM', default="[3,3]")
    Parser.add_argument('--weight', default=1)

    args = Parser.parse_args(rospy.myargv()[1:])
    source_location = [float(i) for i in args.start_location[1:-1].split(',')]
    goal_location = args.goal_location
    goal_location = [float(i) for i in goal_location[1:-1].split(',')]
    print(goal_location)
    print(source_location)
    r = float(args.robot_radius)
    c = float(args.clearance)

    step_size = int(args.step_size)
    thresh = float(args.thresh)
    goal_threshold = float(args.goal_threshold)
    wheel_length = float(args.wheel_length)
    rpm = [float(i) for i in args.RPM[1:-1].split(',')]
    Ur, Ul = rpm[0], rpm[1]
    print(Ur, Ul)
    wheel_radius = float(args.wheel_radius)
    weight = float(args.weight)

    startPose(source_location)

    path_planner = astar.Astar(
        source_location, goal_location, step_size=step_size,
        goal_threshold=goal_threshold, width=10, height=10, thresh=thresh, r=r,
        c=c, wheel_length=wheel_length, Ur=Ur, Ul=Ul,
        wheel_radius=wheel_radius, weight=weight,
        displayExploration=int(args.displayExploration),
        displayPath=int(args.displayPath))
    rospy.loginfo("Finding Optimal Path using A* Star Algorithm")
    path_planner.findOptimalPath()
    rospy.loginfo("Path found, moving turtlebot...")
    driveTurtlebot(args, path_planner)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
