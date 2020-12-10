#!/usr/bin/env python
# VITA, EPFL
from __future__ import division
import numpy as np
import scipy.optimize as opt
import math
import time
import rospy

class MobileRobot:
    def __init__(self, wheel_base, v_max):
        self.wheel_base = wheel_base
        self.v_max = v_max
        self.w_max = v_max / wheel_base


class MPC:
    def __init__(self, mobile_robot, dt_control, prediction_horizon):

        self.dt = dt_control
        self.N = int(prediction_horizon/dt_control)
        self.x0 = [0.0] * 3
        self.xref = [0] * self.N
        self.predicted_states = []
        self.u_total_prev = [1.0] * self.N + [0.001] * self.N
        self.debug_activated = False
        self.con1 = {"type": "ineq", "fun": self.constraints_delta_u}
        self.x_error = 0.0
        self.y_error = 0.0
        self.heading_error = 0.0

        ####### Weights:

        # w_Q + w_dR = 1
        w_Q = 0.1
        w_dR = 0.9

        # w_Q_ex + w_Q_ey + w_Q_epsi + w_Q_ev = 1
        w_Q_ex = 0.25
        w_Q_ey = 0.25
        w_Q_eheading = 0.5


        # w_dR_steer + w_dR_torque = 1
        w_dR_v = 0.2
        w_dR_w = 0.8


        ####### Maximum admissible values:

        # Maximum admissible error in the states:
        ex_max = 0.2 # m
        ey_max = 0.2 # m
        eheading_max = 5.0 * (math.pi/180.0) # degrees --> rad

        # Maximum output rate:
        self.dv_max = 0.5 * self.dt # m/s
        self.dw_max = 1.0 * self.dt # rad/s


        ####### Q and dR matrices:
        Q_ex = w_Q_ex / (ex_max**2)
        Q_ey = w_Q_ey / (ey_max**2)
        Q_epsi = w_Q_eheading / (eheading_max**2)

        self.Q = w_Q*np.array([Q_ex,Q_ey,Q_epsi])

        dR_v = w_dR_v / (self.dv_max**2)
        dR_w = w_dR_w / (self.dw_max**2)

        self.dR = w_dR*np.array([dR_v,dR_w])

        ####### Boundaries:
        self.bnds_v = ((-mobile_robot.v_max, mobile_robot.v_max), ) * self.N
        self.bnds_w = ((-mobile_robot.w_max, mobile_robot.w_max), ) * self.N
        self.bnds = self.bnds_v + self.bnds_w


    def get_next_state(self, x0, u0):

        dt = self.dt

        x, y, heading = x0
        v, w = u0

        # Inverse kinematics of a two-wheel robot --> Local coordinates
        x_n = x + v * dt * np.cos(w*dt)
        y_n = y + v * dt * np.sin(w*dt)
        heading_n = heading + dt * w

        state = np.array([x_n, y_n, heading_n])
        return state


    def objective_function(self, u):

        states = []

        u_v = u[:self.N]
        u_w = u[self.N:]

        u_r = np.zeros((self.N,2))

        u_r[:,0] = u_v
        u_r[:,1] = u_w

        u_tot = u_r

        state = self.x0
        states.append(np.array(state))

        for uu in u_r:
            state = self.get_next_state(state, uu)
            states.append(state)

        self.predicted_states = states
        xy_ref = [np.array([ref[0],ref[1]]) for ref in self.xref]
        xy_state = [np.array([state[0], state[1]]) for state in states]

        ds = [xy_state[e] - xy_ref[e] for e in range(len(xy_ref))]

        heading_state = np.array([np.arctan2(d[1],d[0]) for d in ds])
        dheading = heading_state - np.array([e[2] for e in self.xref])

        err_x = [np.linalg.norm(ds[e])*np.cos(dheading[e]) for e in range(len(ds))]
        err_y = [np.linalg.norm(ds[e])*np.sin(dheading[e]) for e in range(len(ds))]
        mse_x = np.sum(np.array(err_x)**2)
        mse_y = np.sum(np.array(err_y)**2)
        mse_heading = np.sum(np.array(np.array([e[2] for e in self.xref]) - np.array([e[2] for e in states]))**2)

        self.x_error = mse_x * self.Q[0]
        self.y_error = mse_y * self.Q[1]
        self.heading_error = mse_heading * self.Q[2]

        mse_pos = mse_x * self.Q[0] + mse_y * self.Q[1]+ mse_heading* self.Q[2]
        mse_du = np.sum(np.dot((u_tot[1:] - u_tot[:-1]) ** 2, self.dR))
        mse = mse_pos + mse_du

        return np.linalg.norm(mse)

    def constraints_delta_u(self, u):

        w = np.array(u[self.N:])
        const1 = abs(w[1:]-w[:-1]) - np.ones(self.N-1) * self.dw_max

        return -const1



def mpc_control_loomo(mpc, x0, xref):
    # Initial variables:
    mpc.x0 = x0
    mpc.xref = xref

    # Optimizer
    res = opt.minimize(mpc.objective_function, mpc.u_total_prev, bounds = mpc.bnds, method = 'SLSQP', constraints = mpc.con1 )

    # Save the interesting values of the solver
    v_cmd = res.x[0]
    w_cmd = res.x[mpc.N]
    control_command = [v_cmd, w_cmd]
    mpc.u_total_prev = res.x

    mse = mpc.objective_function(mpc.u_total_prev)
    mpc.debug_activated = False

    if mpc.debug_activated:
        print("-------------------NEW CONTROL COMMAND----------------------")
        #rospy.loginfo("Message Control" + str(res.message))
        #rospy.loginfo("CONTROL TIME: " + str(end_time - start_time))
        rospy.loginfo("CONTROL Mean Squared Error Total: " + str(mse))
        rospy.loginfo("CONTROL Mean Squared Error x: " + str(mpc.x_error))
        rospy.loginfo("CONTROL Mean Squared Error y: " + str(mpc.y_error))
        rospy.loginfo("CONTROL Mean Squared Error heading: " + str(mpc.heading_error))
        #rospy.loginfo("STATE: " + str(x0))
        #rospy.loginfo("PLANNER: " + str(xref))
        #rospy.loginfo("ACTUAL CONTROL COMMANDS: " + str(control_command))
        #rospy.loginfo("NEXT CONTROL COMMANDS: " + str(mpc.u_total_prev))
        #rospy.loginfo("PREDICTED STATES: " + str(mpc.predicted_states))
        #print("-----------------------------------------------------------")

    return [control_command, mpc.predicted_states]


if __name__ == '__main__':

    mpc_control_loomo(MPC(MobileRobot(0.5, 2), ))