import obstacle
import math
from heapq import heappush, heappop


class Astar():
    def __init__(
            self,
            source_location,
            goal_location,
            theta_step=30,
            step_size=1,
            goal_threshold=0.1,
            width=10,
            height=10,
            thresh=0.5,
            r=0.1,
            c=0.1,
            wheel_length=0.038,
            Ur=2,
            Ul=2,
            wheel_radius=2,
            dt=0.1,
            dtheta=0,
            weight=1,
            displayExploration=0,
            displayPath=1):
        self.source_location = source_location
        self.goal_location = goal_location
        # nodeData = [ x , y , a , cost ]
        self.nodeData = []
        self.weight = weight
        self.Data = []
        self.allData = []
        self.theta_step = theta_step
        self.dt = dt
        self.dtheta = dtheta
        self.wheel_radius = wheel_radius
        self.wheel_length = wheel_length
        self.Ur = Ur
        self.Ul = Ul
        self.displayExploration = displayExploration
        self.displayPath = displayPath
        self.step_size = step_size
        self.goal_threshold = goal_threshold
        self.path = []
        self.trace_index = []
        self.goal_reached = False
        self.actions = [[0, self.Ur],
                        [self.Ul, 0],
                        [0, self.Ul],
                        [self.Ur, 0],
                        [self.Ul, self.Ur],
                        [self.Ur, self.Ul],
                        [self.Ur, self.Ur],
                        [self.Ul, self.Ul]]
        self.actionSet = []
        self.obstacle = obstacle.Obstacle(
            width, height, r=r, c=c, thresh=thresh, actions=self.actions,
            wheel_length=self.wheel_length, wheel_radius=self.wheel_radius)

    def actionData(self, present_node):
        index = 0
        self.actionSet = []

        for action_vector in self.actions:
            t = 0
            dt = 0.1
            x, y, a = present_node[1], present_node[2], present_node[3]
            # a = angle
            a = 3.14 * a / 180.0
            costToCome = 0
            for i in range(10):
                t = t + dt
                xnew = 0.5 * (self.wheel_radius) * (
                    action_vector[0] + action_vector[1]) * math.cos(a) * dt
                ynew = 0.5 * (self.wheel_radius) * (
                    action_vector[0] + action_vector[1]) * math.sin(a) * dt
                x += xnew
                y += ynew
                a += (self.wheel_radius / self.wheel_length) * \
                    (action_vector[1] - action_vector[0]) * dt
                costToCome += math.sqrt(xnew**2 + ynew**2)
            a = 180 * (a) / 3.14
            self.actionSet.append([x, y, a, costToCome, index])
            index += 1
        return

    def checkObstacle(self):

        if not self.obstacle.isObstacle(
                self.goal_location[0],
                self.goal_location[1]):
            print("Goal location is on obstacle!")
            return False
        elif not self.obstacle.isObstacle(self.source_location[0], self.source_location[1]):
            print("Start location is on obstacle!")
            return False
        else:

            cost = math.sqrt(
                (self.source_location[0] - self.goal_location[0]) ** 2 +
                (self.source_location[1] - self.goal_location[1]) ** 2)
            heappush(
                self.Data,
                [cost, self.source_location[0],
                 self.source_location[1],
                 self.source_location[2],
                 0])

            self.nodeData.append(
                [self.source_location[0],
                 self.source_location[1],
                 self.source_location[2],
                 0])
            return True

    def heuristic_dist(self, current):
        h = self.weight * math.sqrt(
            (current[1] - self.goal_location[0]) ** 2 +
            (current[2] - self.goal_location[1]) ** 2)
        return h

    def checkGoal(self, current):
        x, y = current[1], current[2]
        if (x - self.goal_location[0])**2 + (
                y - self.goal_location[1])**2 <= (self.goal_threshold)**2:
            return True
        else:
            return False

    def backTracePath(self, present_node):
        track = []
        trace_index = []
        current_node = present_node[:4]
        track.append(current_node)
        trace_index.append(0)
        while current_node[1:] != self.source_location:
            l, ind = self.obstacle.getExploredList(current_node)
            current_node = list(l)
            track.append(current_node)
            trace_index.append(ind)
        track.reverse()
        trace_index.reverse()
        return track, trace_index

    def findOptimalPath(self):
        count = 0
        if self.checkObstacle():
            while len(self.Data) > 0:
                count += 1
                present_node = heappop(self.Data)
                previous_cost, previous_cost_to_come = present_node[0], present_node[4]
                if self.checkGoal(present_node):
                    self.goal_reached = True
                    print("Goal reached")
                    self.path, self.trace_index = self.backTracePath(
                        present_node)
                    if self.displayExploration:
                        self.obstacle.explorationPlot()

                    if self.displayPath:
                        self.obstacle.plotPath(self.path, self.trace_index)
                    return
                self.actionData(present_node)
                for action_vector in self.actionSet:

                    new_X = action_vector[0]
                    new_Y = action_vector[1]
                    new_A = action_vector[2]
                    new_node = [0, new_X, new_Y, new_A, 0]
                    new_cost_to_come = previous_cost_to_come + action_vector[3]
                    new_node[4] = new_cost_to_come
                    cost_to_go = self.heuristic_dist(new_node)
                    if self.obstacle.isObstacle(new_X, new_Y):
                        if not self.obstacle.isExplored(new_node):
                            present_node[0] = new_cost_to_come
                            self.obstacle.exploredList(
                                new_node, present_node[: 4],
                                action_vector[4])
                            new_node[0] = new_cost_to_come + cost_to_go
                            heappush(self.Data, new_node)

                        # check visited nodes
                        else:
                            previous_visited, _ = self.obstacle.getExploredList(
                                new_node)
                            previous_cost = previous_visited[0]
                            if previous_cost > new_cost_to_come:
                                present_node[0] = new_cost_to_come
                                self.obstacle.exploredList(
                                    new_node, present_node[: 4],
                                    action_vector[4])
        print("Unable to reach goal location!!")
        return
