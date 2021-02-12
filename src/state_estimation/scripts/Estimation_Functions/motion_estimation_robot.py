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

        x, y, heading, v, a, w = x

        ux = (1/w**2) * \
             ((v*w + a*w*dt)* \
              np.sin(heading + w*dt) + a*np.cos(heading + w*dt) - v*w*np.sin(heading) - np.cos(heading)*a)
        uy = (1/w**2) * \
             ((-v*w - a*w*dt)* np.cos(heading + w*dt) \
              + a*np.sin(heading + w*dt) + v*w*np.cos(heading) - a*np.sin(heading))

        x = x + ux
        y = y + uy
        heading = heading + dt*w
        v = v + dt*a
        a = a
        w = w

        # Calculate the Jacobian of the Dynamic Matrix A
        # see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
        a13 = float((1/w**2) * (a*np.sin(heading)- a*np.sin(dt*w+heading) - v*w*np.cos(heading) + (dt*a*w + v*w)*np.cos(dt*w+heading)))
        a23 = float((1/w**2) * (-a*np.cos(heading)+ a*np.cos(dt*w+heading) - v*w*np.sin(heading) - (-dt*a*w - v*w)*np.sin(dt*w+heading)))

        a14 = float((1/w**2) * (-w*np.sin(heading) + w*np.sin(dt*w+heading)))
        a24 = float((1/w**2) * (w*np.cos(heading) - w*np.cos(dt*w+heading)))

        a15 = float((1/w**2) * (dt*w*np.sin(dt*w+heading)-np.cos(heading)+np.cos(dt*w + heading)))
        a25 = float((1/w**2) * (-dt*w*np.cos(dt*w+heading)-np.sin(heading)+np.sin(dt*w + heading)))

        a161 = float((1/w**2) * (-dt*a*np.sin(dt*w+heading)+dt*(dt*a*w+v*w)*np.cos(dt*w+heading)-v*np.sin(heading)+(dt*a+v)*np.sin(dt*w+heading)))
        a162 = float((1/w**3) * (-a*np.cos(heading)+a*np.cos(dt*w+heading)-v*w*np.sin(heading)+(dt*a*w+v*w)*np.sin(dt*w+heading)))
        a16 = a161-2*a162

        a261 = float((1/w**2) * (dt*a*np.cos(dt*w+heading)-dt*(-dt*a*w-[3]*w)*np.sin(dt*w+heading)+v*np.cos(heading)+(-dt*a-v)*np.cos(dt*w+heading)))
        a262 = float((1/w**3) * (-a*np.sin(heading)+a*np.sin(dt*w+heading)+v*w*np.cos(heading)+(-dt*a*w-v*w)*np.cos(dt*w+heading)))
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
        hx = np.matrix([[float(x)],
                        [float(y)],
                        [float(heading)],
                        [float(v)],
                        [float(a)],
                        [float(w)]])

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


    def kalman(self, mes):
        measurements = np.matrix([mes]).T

        if self.first_mes:
            self.x = measurements
            self.first_mes = False

        self.x, self.P = self.kalman_update(self.x, measurements, self.P, self.Q, self.R, self.I, self.dt)

        return self.x.A1