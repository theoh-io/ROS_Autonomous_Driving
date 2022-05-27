#!/usr/bin/env python3
# VITA, EPFL
import rospy
from msg_types.msg import Position, PositionArray, TrajectoryArray, State, StateArray
from Path_planning_Functions import RRT_star, CHUV_Planner
import time
from tools import classconverter, classes, transformations, utilities


class Sender(object):

    def __init__(self):
        self.pub_array_path_local = rospy.Publisher('/Path_planning/desired_states_local', StateArray, queue_size=1)
        self.pub_goal_local = rospy.Publisher('/Visualization/goal_local', Position, queue_size=1)


    def send(self, desired_path, objects_now_local, x0, goal_local):
        array_path_local = classconverter.list2StateArray(desired_path)
        array_objects = classconverter.list2TrajectoryArray(objects_now_local)
        state_planner = classconverter.list2State(x0)
        goal_local = classconverter.list2Position(goal_local)
        array_path_local.sync_predictions = array_objects
        array_path_local.initial_state = state_planner

        # Publish "/desired_trajectory" commands:
        self.pub_array_path_local.publish(array_path_local)
        self.pub_goal_local.publish(goal_local)


def callback_prediction(data): # data is TrajectoryArray --> [[[x_11,y_11],[x_12,y_12]],[[x_21,y_21],[x_22,y_22]]]

    global array_predictions

    array_predictions = classconverter.TrajectoryArray2list(data)


def callback_estimation(data): # data is State --> [x, y, heading]

    global state

    state = classconverter.State2list(data)


def callback_mapping(data):

    global array_mapping

    array_mapping = classconverter.PositionArray2TrajectoryArraylist(data)


def main():
    # Initialize ROS
    rospy.init_node("path_planning")
    dt_path_planning = rospy.get_param("/dt_path_planning")
    rate = rospy.Rate(int(1/dt_path_planning))
    sub_estimation = rospy.Subscriber('/State_Estimation/estimated_state', State, callback_estimation, queue_size = 1)
    prediction_activated = rospy.get_param("/prediction_activated")
    mapping_activated = rospy.get_param("/mapping_activated")
    PATH_PLANNING_FUNCTION = rospy.get_param("/PATH_PLANNING_FUNCTION")

    if prediction_activated:
        sub_prediction = rospy.Subscriber('/Prediction/predicted_trajectories_global', TrajectoryArray, callback_prediction, queue_size = 1)
        
    if mapping_activated:
        sub_mapping = rospy.Subscriber('/State_Estimation/map_global', PositionArray, callback_mapping, queue_size = 1)

    if not prediction_activated and not mapping_activated:
        ip_address = rospy.get_param("/ip_address_robot")
        socket3 = classes.SocketLoomo(8083, dt_path_planning/4, ip_address)

    sender = Sender()

    # Initialize path planning parameters
    dt_control = rospy.get_param("/dt_control")
    speed = rospy.get_param("/speed")
    wheel_base = rospy.get_param("/wheel_base") # m
    v_max = rospy.get_param("/v_max") # m/s
    time_horizon_control = rospy.get_param("/time_horizon_control")
    N = int(time_horizon_control/dt_control)
    num_person = int(rospy.get_param("/num_person"))-1
    loomo = classes.MobileRobot(wheel_base, v_max)

    # Person-Following algorithm initialization
    if PATH_PLANNING_FUNCTION == "CHUV":
        robot_position = rospy.get_param("/robot_position")
        planner_type = rospy.get_param("/planner_type")
        planner_class = CHUV_Planner.CHUV_Planner(loomo, speed, N, dt_control, robot_position)
        goal_local = [0.0, 0.0]

    # Obstacle/Human avoidance algorithm initialization
    elif PATH_PLANNING_FUNCTION == "Default":
        goal_x = rospy.get_param("/goal_x")
        goal_y = rospy.get_param("/goal_y")
        goal_global = [goal_x, goal_y]
        work_area = [rospy.get_param("/workarea_x_min"), rospy.get_param("/workarea_x_max"), rospy.get_param("/workarea_y_min"), rospy.get_param("/workarea_y_max")]

    # Initialize path planning variables
    global array_predictions, array_mapping, state

    array_predictions = []
    array_mapping = []
    state = [0.0, 0.0, 0.0]
    objects_now_local = []
    path = []
    iterations = 0

    rospy.loginfo("Path Planning Node Ready")
    rospy.sleep(2.)

    while not rospy.is_shutdown():

        if not prediction_activated and not mapping_activated:
            # Receive detection positions (x, y) in relation to the Loomo
            socket3.receiver()

            if socket3.received_ok:
                positions = [socket3.received_data_unpacked]
                array_predictions = []

                for idx,position in enumerate(positions):

                    if position[0]!=0.0:
                        array_predictions.append([list(position)+[idx+1]+[0.0]])

        start = time.time()
        x0 = state
        array_total_global = array_predictions + array_mapping
        
        # Actual detection movement prediction
        if len(array_total_global) > 0:
            objects_now_local = transformations.Global_to_Local_prediction(x0, array_total_global)

        # CHUV Planner
        if PATH_PLANNING_FUNCTION == "CHUV" and len(objects_now_local)>0:

            if planner_type == "straight":
                path = planner_class.path_planning_straight(array_total_global[num_person], x0)
            
            else:
                path, goal_local = planner_class.path_planning_curvilinear(array_total_global[num_person], x0)

        # Obstacle avoidance path calculation
        elif PATH_PLANNING_FUNCTION == "Default":
            goal_local = transformations.Global_to_Local(x0, [goal_global], True)[0]
            path, goal_local = RRT_star.planner_rrt_star(loomo, objects_now_local, speed, dt_control, goal_local, N, work_area, prediction_activated=prediction_activated)

        if len(path)>=N and iterations >= 0:
            # Send commands to the ROS Structure
            iterations = 0
            sender.send(path, objects_now_local, x0, goal_local)

        # Calculate node computation time
        computation_time = time.time() - start

        if computation_time > dt_path_planning:
            rospy.logwarn("Path planning computation time higher than node period by " + str(computation_time-dt_path_planning) + " seconds")

        objects_now_local = []
        iterations += 1

        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        pass

