'''
Core code for ekf class

'''
import numpy as np
import math

class EKF:
    def __init__(self, x, P, Q, R, H):
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R
        self.H = H

    def predict(self, v, omega, dt):
        # Predict using the simulated imu and encoder

        # Define F as the Jacobian of the Dynamics (discrete time)
        theta = self.x[2]
        F = np.array([
        [1, 0, -v * math.sin(theta) * dt],
        [0, 1,  v * math.cos(theta) * dt],
        [0, 0, 1]
        ])

        # Update State
        self.x = diff_drive_model(self.x, v, omega, dt)
        # Update Covariance
        self.P = F @ self.P @ F.T + self.Q

        return self.x, self.P
    
    def update(self, z):
        # Update state estimation using simulated GPS values
        
        # Kalman Gain
        K = (self.P @ self.H.T) @ np.linalg.inv((self.H @ self.P @ self.H.T + self.R))

        # Update estimate with gps position (x,y) measurement
        self.x = self.x + K @ (z - self.H @ self.x)

        # Update uncertainty
        I = np.eye(3)
        self.P = (I - K@self.H) @ self.P

        return self.x, self.P
    

def diff_drive_model(state, v_inp:float, w_inp:float, dt:float):
    '''

    state for differential drive robot: [ x, y, theta]
    inputs are v_inp and w_inp

    '''
    x = state[0]
    y = state[1]
    theta = state[2]

    xdot = v_inp * math.cos(theta)
    ydot = v_inp * math.sin(theta)
    thetadot = w_inp

    x_new = x + xdot*dt
    y_new = y + ydot*dt
    theta_new = theta + thetadot*dt

    # Wrap the angle from -pi to pi
    theta_new = (theta_new + np.pi) % (2*np.pi) - np.pi

    state = np.array([x_new, y_new, theta_new])

    return state