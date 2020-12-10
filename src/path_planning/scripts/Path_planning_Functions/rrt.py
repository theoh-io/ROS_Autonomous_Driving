"""
Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)
author: AtsushiSakai(@Atsushi_twi)
"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np


class RRT(object):
    """
    Class for RRT planning
    """

    class Node(object):
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, mobile_robot, start, goal, obstacle_list, rand_area, prediction_activated, speed, expand_dis=0.2, path_resolution=0.25, goal_sample_rate=5, max_iter=500):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand_x = rand_area[0]
        self.max_rand_x = rand_area[1]
        self.min_rand_y = -1
        self.max_rand_y = 1
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.current_objects = obstacle_list[0]
        self.node_list = []
        self.prediction_activated = prediction_activated
        self.speed = speed
        self.max_w = mobile_robot.w_max
        self.dt = self.expand_dis/self.speed



    def steer(self, from_node, to_node, extend_length=float("inf"), extend_theta=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        if abs(theta) > extend_theta:
            theta = np.sign(theta)*extend_theta

        n_expand = int(math.floor(extend_length / self.path_resolution))

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand_x, self.max_rand_x),
                random.uniform(self.min_rand_y, self.max_rand_y))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, [i for i in node.path_y], "-g")
                plt.axis([0, 10, -5, 5])

        for e in self.current_objects:
            self.plot_circle(e[0], e[1], e[2])

        plt.axis("equal")
        plt.axis([0, 10, -5, 5])
        plt.grid(True)
        plt.pause(0.001)


    def check_collision(self, node, obstacleList, prediction_activated, current_time):

        if node is None:
            return False

        if not prediction_activated:
            current_object_position = obstacleList[0]

        else:
            i = 0
            t_min = 100
            for (ox, oy, size, time) in obstacleList:
                t = current_time - time

                if i >= len(obstacleList):
                    break

                elif abs(t) < abs(t_min):
                    t_min = t
                    current_object_position = []
                    current_object_position.append(obstacleList[i])

                elif abs(t) == abs(t_min):
                    current_object_position.append(obstacleList[i])

                i = i + 1

        self.current_objects = current_object_position

        for (ox, oy, size, time) in current_object_position:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # collision


        return True  # safe

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
