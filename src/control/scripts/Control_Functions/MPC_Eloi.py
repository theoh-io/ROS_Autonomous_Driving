import numpy as np
import math as mt
from scipy.optimize import minimize
import datetime


class MPC_model:
    
    # PARAMETERS FOR THE MPC MODEL
    HORIZON_N = 7
    v_max = 0.5
    w_max = 0.5
    path_idx = 0

    # WEIGHT OF ERRORS
    # MEAN SQUARE ERROR BETWEEN POINTS
    q_MSE = 0.3
    max_err_Y = 0.2 * 0.2
    max_err_X = 0.3 * 0.3
    # WEIGHT OF DU_VECTOR
    q_dRs = 0.70
    # WEIGHT OF U_VECTOR
    q_Rs = 0.00

    max_incr_D = 0.5
    max_incr_steer = mt.pi/18
    R_dU = np.array([[0.5 / (max_incr_D)**2, 0.0],
                     [0.0, 0.5 / (max_incr_steer)**2]])

    R_U = np.array([[0.0, 0],
                    [0, 0.0]])

    # x = [X, Y, Phi, V_x, V_y, r] with (X, Y) the car's position, Phi the position and heading in global coordinates,
    # V_x and V_y hor/vert velocity and r the yaw rate r
    # u = [D, steering] with delta the steering angle and D the driving command

    guess_v = [0.5 for i in range(HORIZON_N)]
    guess_w = [0.0 for i in range(HORIZON_N)]
    first_guess = guess_v + guess_w

    previous_states = np.array(first_guess)
    path = np.empty(HORIZON_N)
    #x0 = np.ones(3)

    def __init__(self):
        self.delta_time = 0.2

        bnds_a = ((-self.v_max, self.v_max),) * (self.HORIZON_N)
        bnds_steer = ((-self.w_max, self.w_max), ) * (self.HORIZON_N)
        bounds = bnds_a + bnds_steer
        self.bounds = bounds

    def F_x(self, D, v_x):

        return self.C_m * D - self.C_r0 - self.C_r2 * (v_x ** 2)

    def MSE(self, x_k, x_k_ref):
        dX = (x_k[0] - x_k_ref[0]) * (x_k[0] - x_k_ref[0]) / self.max_err_X
        dY = (x_k[1] - x_k_ref[1]) * (x_k[1] - x_k_ref[1]) / self.max_err_Y

        return self.q_MSE * (dX + dY) / 2

    def error_R(self, u, dU):

        return self.q_Rs * np.matmul(np.matmul(u.T, self.R_U), u) + self.q_dRs * np.matmul(np.matmul(dU.T, self.R_dU), dU)

    def f_next_state(self, x_k, u_k):
        #derivative of speed is driving command projected on the [breaking_max; acel_max] interval
        first = x_k[0] + self.delta_time * (u_k[0] * mt.cos(x_k[2]))
        second = x_k[1] + self.delta_time * (u_k[0] * mt.sin(x_k[2]))
        third = x_k[2] + self.delta_time * u_k[1]
        to_ret = np.array([first, second, third])

        return to_ret


    def big_fun(self, u_ks):

        #reshaping u_ks array
        u_D = u_ks[:self.HORIZON_N]
        u_steering = u_ks[self.HORIZON_N:]
        u_k = np.zeros((self.HORIZON_N,2))
        u_k[:,0] = u_D
        u_k[:,1] = u_steering      

        ## we initialize random U-vector 
        dU = u_k[1:] - u_k[:-1]
        
        # we calculate the next HORIZON_N states of the car according to those Us
        new_state = np.ones((self.HORIZON_N, 3))
        new_state[0] = self.x0

        for i in range(1, self.HORIZON_N, 1):
            new_state[i] = self.f_next_state(new_state[i - 1], u_k[i])

        self.new_state = new_state

        # given the computed states, we compute the error between the wanted position of the car (path) and those states
        error = 0

        for i in range(self.HORIZON_N - 1):
           error += self.MSE(new_state[i], self.path[i]) + self.error_R(u_k[i], dU[i])

        return error


    def run_MPC(self, x0):
        start = datetime.datetime.now()
        
        self.x0 = x0

        result = minimize(self.big_fun, self.previous_states, bounds=self.bounds, method='SLSQP')
        
        self.previous_states = np.array(result.x).reshape((self.HORIZON_N,2))

        to_return = np.array([result.x[0], result.x[self.HORIZON_N]])
        self.previous_good_state = to_return

        end = datetime.datetime.now()
        duration = end - start

        return to_return, self.new_state

    def acquire_path(self, path):
        self.path = path
        self.path_idx = 0


    def shift_path(self):
        self.path_idx += 1
        last = self.path[len(self.path) - 1]
        for i in range(len(self.path) - 1):
            self.path[i] = self.path[i+1]
        self.path[len(self.path) - 1] = last

    def add_point(self, point):
        self.path.append(point)
        
    def calc_distance(self, next, curr):
        """Computes the distance between two points

        Args:
            next (pair): end point of vector
            curr (pair): starting point of vector

        Returns:
            double: the distance between the given points
        """
        diffX = (next[0] - curr[0]) * (next[0] - curr[0])
        diffY = (next[1] - curr[1]) * (next[1] - curr[1])
        return mt.sqrt(diffX + diffY)
