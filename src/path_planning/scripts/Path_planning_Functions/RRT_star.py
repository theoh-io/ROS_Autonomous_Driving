#!/usr/bin/env python
# VITA, EPFL
"""
Path planning Sample Code with RRT*
author: Atsushi Sakai(@Atsushi_twi)
"""
from scipy.interpolate import UnivariateSpline
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath('/home/cconejob/StudioProjects/socket-loomo/src/perception/scripts/tools')))
from tools import classconverter, classes, transformations, utilities

try:
    from rrt import RRT
    import rrt_with_pathsmoothing

except ImportError:
    raise

visualization = False

class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super(RRTStar.Node,self).__init__(x, y)
            self.cost = 0.0

    def __init__(self, mobile_robot, start, goal, obstacle_list, rand_area, prediction_activated, speed, expand_dis=0.2, path_resolution=0.2, goal_sample_rate=20, max_iter=500, connect_circle_dist=10.0, search_until_max_iter=False):
        """
        Setting Parameter
        start:Start Position [x,y]c
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        super(RRTStar,self).__init__(mobile_robot, start, goal, obstacle_list, rand_area, prediction_activated, speed, expand_dis, path_resolution, goal_sample_rate, max_iter)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter

    def planning(self, animation=False):
        """
        rrt star path planning
        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis, self.max_w*self.dt)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                            math.hypot(new_node.x-near_node.x,
                                       new_node.y-near_node.y)

            current_distance = self.node_list[nearest_ind].cost

            current_time = current_distance/self.speed

            if self.check_collision(new_node, self.obstacle_list, self.prediction_activated, current_time):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(new_node, near_inds, current_time)

                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds, current_time)
                    self.node_list.append(node_with_updated_parent)

                else:
                    self.node_list.append(new_node)

            if animation:
                self.draw_graph(rnd)

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node(current_time)
                if last_index is not None:
                    return self.generate_final_course(last_index)

        rospy.logwarn("planner reached max iteration")

        last_index = self.search_best_goal_node(current_time)
        if last_index is not None:
            return self.generate_final_course(last_index)

        return []

    def choose_parent(self, new_node, near_inds, current_time):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node
            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)

            if t_node and self.check_collision(t_node, self.obstacle_list, self.prediction_activated, current_time):
                costs.append(self.calc_new_cost(near_node, new_node))

            else:
                costs.append(float("inf"))  # the cost of collision node
        
        min_cost = min(costs)

        if min_cost == float("inf"):
            rospy.logwarn("There is no good path.")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self, current_time):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis]
        safe_goal_inds = []

        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)

            if self.check_collision(t_node, self.obstacle_list, self.prediction_activated, current_time):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])

        for i in safe_goal_inds:

            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the tree that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))

        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]

        return near_inds

    def rewire(self, new_node, near_inds, current_time):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree
                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)

            if not edge_node:
                continue
            
            edge_node.cost = self.calc_new_cost(new_node, near_node)
            no_collision = self.check_collision(edge_node, self.obstacle_list, self.prediction_activated, current_time)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)

        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:

            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


def distance_AB(A, B):

    return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


def triangleArea(a,b,c):

    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def MPC_planner(points, v , t):
    e = v*t
    plan = [[0.0, 0.0, 0.0, 0.0]]
    dist = 0.0
    it = 2
    x_ant = 0.0
    y_ant = 0.0
    heading_ant = 0.0

    for i in range(1,80):

        while dist < e and it < len(points):
            distAB = distance_AB(points[it-2], points[it-1])
            dist = dist + distAB
            it = it + 1

        if dist >= e:
            it = it - 1
            d = dist - e
            prop = d/distAB
            x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
            y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
            heading = math.atan2((y - y_ant),(x - x_ant))
            plan.append([x, y, heading, v])
            dist = d
            x_ant = x
            y_ant = y
            it = it + 1

    return plan


def MPC_Planner_restrictions(mobile_robot, points, v, t):
    e = v*t
    plan = [[0.0, 0.0, 0.0, 0.0]]
    dist = 0.0
    it = 2
    x_ant = 0.0
    y_ant = 0.0
    heading_ant = 0.0

    for i in range(1,80):
        
        while dist < e and it < len(points):
            distAB = distance_AB(points[it-2], points[it-1])
            dist = dist + distAB
            it = it + 1

        if dist >= e:
            it = it - 1
            d = dist - e
            prop = d/distAB
            x = points[it-1][0] - prop*(points[it-1][0] - points[it-2][0])
            y = points[it-1][1] - prop*(points[it-1][1] - points[it-2][1])
            heading = math.atan2((y - y_ant),(x - x_ant))

            if abs(heading-heading_ant) > mobile_robot.w_max * t:
                heading = heading_ant + np.sign(heading-heading_ant) * mobile_robot.w_max * t
                x = x_ant + e * np.cos(heading)
                y = y_ant + e * np.sin(heading)

            plan.append([x, y, heading, v])
            dist = d
            x_ant = x
            y_ant = y
            heading_ant = heading
            it = it + 1

    return plan




def add_correction(mobile_robot, dt, start, goal):
    heading = start
    error_angle = goal-start
    added_points = [[0.0, 0.0, heading, 0.0]]

    if error_angle > 0.0:
        w_max = mobile_robot.w_max/2

    else:
        w_max = -mobile_robot.w_max/2

    i = 0

    while abs(error_angle) > 10*math.pi/180 and i<50:
        heading_n = heading + dt * w_max
        added_points.append([0.0, 0.0, heading_n, 0.0])
        heading = heading_n
        error_angle = heading_n - goal
        i = i + 1

    return added_points


def planner_rrt_star(mobile_robot, array_predictions, speed, dt_control, goal=[0.0, 0.0], N=1, prediction_activated=False):

    obstacle_list = []
    are_obstacles = True

    for e1 in array_predictions:

        for i,e2 in enumerate(e1):

            if not prediction_activated and i>0:
                pass

            elif len(e2)>0:
                obstacle_list.append((e2[0], e2[1], 0.5, e2[2]))

    if len(obstacle_list) < 1:
        are_obstacles = False

    if are_obstacles:
        # Set Initial parameters
        rrt_star = RRTStar(mobile_robot, start=[0, 0], goal=[goal[0], goal[1]], obstacle_list=obstacle_list, rand_area=[0, goal[0]], prediction_activated=prediction_activated, speed=speed)

    path = []
    plt.clf()

    if goal[0] != 0.0:
        m = goal[1]/goal[0]

    else:
        m = 10000.0

    x_list = np.linspace(0.0, goal[0], num=20)
    y_list = m * x_list

    for i in range(len(x_list)):
        path.append([x_list[i], y_list[i]])

    no_obstacles_in_path = True

    if are_obstacles:
        i = 0

        while no_obstacles_in_path and i<(len(path)-1):
            no_obstacles_in_path = rrt_with_pathsmoothing.line_collision_check(path[i], path[i+1], obstacle_list, i*dt_control)
            i = i + 1
    
    goal_angle = 0.0

    if len(path)>0:
        goal_angle = math.atan2((path[1][1]),(path[1][0]))
    
    smooth_path = path

    if not no_obstacles_in_path:
        rospy.logwarn("Found obstacle in path")
        path = rrt_star.planning(animation=False)

        if visualization:
            rrt_star.draw_graph()

        if path is None:
            planner = []
            return planner

        else:
            x_ant = -1.0
            x_list = []
            y_list = []
            path = path[1:][::-1]
            rotated_path = path
            if len(path)>0:
                goal_angle = math.atan2((path[1][1]-path[0][1]),(path[1][0]-path[0][0]))
                rotated_path = transformations.rotate(path, goal_angle)

            

            for e in rotated_path:

                if x_ant<e[0]:
                    x_list.append(e[0])
                    y_list.append(e[1])
                    x_ant = e[0]

    if len(x_list)>4 and not no_obstacles_in_path:
        smooth_path_rotated = []
        path_spline = UnivariateSpline(x_list, y_list, k=4)
        path_spline.set_smoothing_factor(0.8)

        for e in x_list:
            smooth_path_rotated.append([e,path_spline(e)])

        smooth_path = transformations.rotate(smooth_path_rotated, -goal_angle)

    mpc_planner = MPC_Planner_restrictions(mobile_robot, smooth_path, speed, dt_control)

    # Correct only the angle if it is too high
    if abs(goal_angle)> 90*math.pi/180:
        rospy.logwarn("Rotation required")
        added_points = add_correction(mobile_robot, dt_control, 0.0, goal_angle)
        mpc_planner = added_points

    # Draw final path
    if visualization:
        plt.plot(0.0, 0.0, "xr")
        plt.plot(goal[0], goal[1], "xr")
        plt.axis([0, 10, -5, 5])

        if are_obstacles:

            for i,e in enumerate(obstacle_list):
                rrt_star.plot_circle(e[0], e[1], e[2])
                plt.annotate('%s' % i,(e[0], e[1]))

        plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
        plt.plot([x for [x, y] in smooth_path], [y for [x, y] in smooth_path], 'b--')
        plt.pause(1)

    return mpc_planner, goal


if __name__ == '__main__':
    planner_rrt_star([(3.0,0.2,0.5,0.0),(6.0,-0.5,0.5,5.0)], 1.0, 0.05, True, [5.0,0.0], True)

