
import numpy as np
import matplotlib.pyplot as plt
from mpc import MPC_model
import random
from pure_pursuit import Pure_model

'''
points = [[0.0, 0.0], 
          [0.13, 0.04], 
          [0.25, 0.22], 
          [0.3, 24.57], 
          [30, 30],
          [38.04, 33.18],
          [48.06, 34.45],
          [57.52, 33.46],
          [66.84, 31.06],
          [77.43, 28.38],
          [82.09, 25.13],
          [87.45, 21.46],
          [93.66, 17.22],
          [100.02, 13.13],
          [105.52, 8.61],
          [111.31, 4.66],
          [119.36, 2.82],
          [126.98, 2.82],
          [133.62, 2.97],
          [143.50, 3.81],
          [151.94, 5.06],
          [164.24, 7.31],
          [175.31, 9.06]]

'''


goal = (3,0)
if goal[0] != 0.0:
    m = goal[1]/goal[0]
else:
    m = 10000.0

x_list = np.linspace(0.0, goal[0], num=40)
y_list = m * x_list

points = list()
for i in range(len(x_list)):
    points.append((x_list[i], y_list[i]))


def display_figure(points, position):

    plt.clf()
    plt.axis([-.1, 4, -1, 1])
    plt.scatter(position[0], position[1], c=10, s=20, zorder=3)

    for i in range(len(points)):
        plt.scatter(points[i][0], points[i][1], zorder=1)
        if(i != 0):
            plt.plot([points[i-1][0], points[i][0]], [points[i-1][1], points[i][1]], 'ro-', zorder=1)


    plt.pause(0.001)


def plot_MPC(x0, points):

    mpc_module = MPC_model()
    final_pos = (points[len(points) - 1][0], points[len(points) - 1][1])
    current_pos = (x0[0], x0[1])

    #initialize path on MPC module 
    mpc_module.acquire_path(points.copy())

    #initialze records if wanted to be printed at the end of the algo.
    current_state = x0
    states = list(x0)
    u_vectors = [[0, 0]]
    dU_vectors = [[0, 0]]
    distance = mpc_module.calc_distance(final_pos, current_pos)


    #running it final goal is reached
    while(distance > 0.1):
        new_u = mpc_module.run_MPC(current_state)

        head = u_vectors[len(u_vectors) - 1]
        new_dU = (new_u[0] - head[0], new_u[1] - head[1])

        u_vectors.append(new_u)
        dU_vectors.append(new_dU)
        new_state = mpc_module.f_next_state(current_state, new_u, new_dU)

        current_pos = (new_state[0], new_state[1])
        states.append(new_state)
        current_state = new_state
        mpc_module.shift_path()

        distance = mpc_module.calc_distance(final_pos, current_pos)
        display_figure(points, current_pos)


        #De-comment to see the updates
        #print("\n NEW ITERATION : \n")
        #print("Distance until final destination :", distance)
        #print("New U vector : ", new_u)
        #print("New DU vector : ", new_dU)
        #print("New state :", new_state)



    #De-comment to see the records
    #print("States : ",states)
    #print("U_vectors : ", u_vectors)
    #print("dU_vectors : ", dU_vectors)

    return 0

plt.show()
display_figure(points, (0,0))

plot_MPC((0, 0, 0, 0, 0, 0), points)


