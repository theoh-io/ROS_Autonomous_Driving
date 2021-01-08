import socket
import sys
import pickle
import time

import numpy as np
import math

class Driverless_Estimation:

    def __init__(self):

        self.numstates = 6
        self.dt = 1.0/0.05

        varGPS = 6.0
        varspeed = 1.0
        varacc = 1.0
        varyaw = 0.9 # Variances on sensor data I had. To be changed for the car when we have the sensors
        self.R = np.diag([varGPS**2, varGPS**2, varyaw**2, varspeed**2,varacc**2, varspeed**2])
        self.I = np.eye(self.numstates)

        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        sPos     = 0.5*8.8*self.dt**2
        sCourse  = 0.1*self.dt
        sVelocity= 8.8*self.dt #  Again, to be changed
        self.Q = np.diag([sPos**2, sPos**2, sCourse**2, sVelocity**2, sCourse**2, sVelocity**2])

        self.first_mes = True


    def kalman_update(self, x, data, P, Q, R, I, dt):

        #x0 is x, x1 is y, x2 is heading, x3 is velocity, x4 is acceleration, x5 is yaw rate


        ux = (1/x[5]**2) * \
             ((x[3]*x[5] + x[4]*x[5]*dt)* \
              np.sin(x[2] + x[5]*dt) + x[4]*np.cos(x[2] + x[5]*dt) - x[3]*x[5]*np.sin(x[2]) - np.cos(x[2])*x[4])
        uy = (1/x[5]**2) * \
             ((-x[3]*x[5] - x[4]*x[5]*dt)* np.cos(x[2] + x[5]*dt) \
              + x[4]*np.sin(x[2] + x[5]*dt) + x[3]*x[5]*np.cos(x[2]) - x[4]*np.sin(x[2]))


        x[0] = x[0] + ux
        x[1] = x[1] + uy
        x[2] = x[2] + dt*x[5]
        x[3] = x[3] + dt*x[4]
        x[4] = x[4]
        x[5] = x[5]

        # Calculate the Jacobian of the Dynamic Matrix A
        # see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
        a13 = float((1/x[5]**2) * (x[4]*np.sin(x[2])- x[4]*np.sin(dt*x[5]+x[2]) - x[3]*x[5]*np.cos(x[2]) + (dt*x[4]*x[5] + x[3]*x[5])*np.cos(dt*x[5]+x[2])))
        a23 = float((1/x[5]**2) * (-x[4]*np.cos(x[2])+ x[4]*np.cos(dt*x[5]+x[2]) - x[3]*x[5]*np.sin(x[2]) - (-dt*x[4]*x[5] - x[3]*x[5])*np.sin(dt*x[5]+x[2])))

        a14 = float((1/x[5]**2) * (-x[5]*np.sin(x[2]) + x[5]*np.sin(dt*x[5]+x[2])))
        a24 = float((1/x[5]**2) * (x[5]*np.cos(x[2]) - x[5]*np.cos(dt*x[5]+x[2])))

        a15 = float((1/x[5]**2) * (dt*x[5]*np.sin(dt*x[5]+x[2])-np.cos(x[2])+np.cos(dt*x[5] + x[2])))
        a25 = float((1/x[5]**2) * (-dt*x[5]*np.cos(dt*x[5]+x[2])-np.sin(x[2])+np.sin(dt*x[5] + x[2])))

        a161 = float((1/x[5]**2) * (-dt*x[4]*np.sin(dt*x[5]+x[2])+dt*(dt*x[4]*x[5]+x[3]*x[5])*np.cos(dt*x[5]+x[2])-x[3]*np.sin(x[2])+(dt*x[4]+x[3])*np.sin(dt*x[5]+x[2])))
        a162 = float((1/x[5]**3) * (-x[4]*np.cos(x[2])+x[4]*np.cos(dt*x[5]+x[2])-x[3]*x[5]*np.sin(x[2])+(dt*x[4]*x[5]+x[3]*x[5])*np.sin(dt*x[5]+x[2])))
        a16 = a161-2*a162

        a261 = float((1/x[5]**2) * (dt*x[4]*np.cos(dt*x[5]+x[2])-dt*(-dt*x[4]*x[5]-[3]*x[5])*np.sin(dt*x[5]+x[2])+x[3]*np.cos(x[2])+(-dt*x[4]-x[3])*np.cos(dt*x[5]+x[2])))
        a262 = float((1/x[5]**3) * (-x[4]*np.sin(x[2])+x[4]*np.sin(dt*x[5]+x[2])+x[3]*x[5]*np.cos(x[2])+(-dt*x[4]*x[5]-x[3]*x[5])*np.cos(dt*x[5]+x[2])))
        a26 = a261-2*a262


        JA = np.matrix([[1.0, 0.0, a13, a14, a15, a16],
                        [0.0, 1.0, a23, a24, a25, a26],
                        [0.0, 0.0, 1.0, 0.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 1.0, dt, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])


        # Project the error covariance ahead
        P = JA*P*JA.T + Q

        # Measurement Update (Correction)
        # ===============================
        # Measurement Function
        hx = np.matrix([[float(x[0])],
                        [float(x[1])],
                        [float(x[2])],
                        [float(x[3])],
                        [float(x[4])],
                        [float(x[5])]])


        JH = np.diag([1.0,1.0,1.0,1.0,1.0,1.0])
        S = JH*P*JH.T + R
        K = (P*JH.T) * np.linalg.inv(S)

        #Z = measurements[:,filterstep].reshape(JH.shape[0],1)
        #Z = data.reshape(JH.shape[0],1)
        Z = data
        y = Z - (hx)
        x = x + (K*y)

        # Update the error covariance
        P = (I - (K*JH))*P

        return (x, P)


    def kalman(self,mes):
        measurements = np.matrix([mes]).T
        
        if self.first_mes:
            self.x = measurements
            self.first_mes = False

        self.x, self.P = self.kalman_update(self.x, measurements, self.P, self.Q, self.R, self.I, self.dt)
        return self.x.A1