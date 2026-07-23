import numpy as np
import math
import random
import matplotlib.pyplot as plt



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

def waypoint_controller(state, waypts, waypt_idx):

    '''

    Simple Feedback controller. Returns v_cmd and w_cmd

    '''
    num_of_waypoints = len(waypts)

    dx = waypts[waypt_idx, 0] - state[0]
    dy = waypts[waypt_idx, 1] - state[1]
    distance = math.sqrt(dx**2 + dy**2)

    theta_des = np.arctan2(dy, dx)
    dtheta = theta_des - state[2]
    dtheta_wrap = (dtheta + np.pi) % (2*np.pi) - np.pi



    Kv = 1.0
    Kw = 1.0
    
    if abs(dtheta_wrap) > np.pi/12:
        v= 0.0
        w = Kw * dtheta_wrap

    else:
        v = Kv * distance
        w = Kw * dtheta_wrap

    # Saturate commands
    v_cmd = np.clip(v, 0, 2.0) # Try NO reverse
    w_cmd = np.clip(w, -1.5, 1.5)

    if distance < 0.1 and waypt_idx<num_of_waypoints-1:
        # Don't allow the index to exceed the waypoint list
        waypt_idx += 1

    if waypt_idx == num_of_waypoints-1 and distance <0.1:
        # If the final waypoint is reached, stop moving
        v_cmd = 0
        w_cmd = 0

    return v_cmd, w_cmd, waypt_idx



def main():

    waypoint_list = np.array([[0,3],[2,1], [4,3], [4,0]])
    waypt_idx = 0
    dt = 0.1
    tf = 40

    timelist = np.arange(0, tf, dt)
    n = len(timelist)



    # initialize the states
    state_true = np.array([0,0,0])
    state_dead_reck = np.array([0,0,0])
    state_ekf = np.array([0,0,0])

    # Lists for storing
    state_true_list = np.zeros((n, 3))
    state_dead_reck_list = np.zeros((n, 3))
    state_ekf_list = np.zeros((n, 3))
    P_x_list = np.zeros(n)
    P_y_list = np.zeros(n)
    P_theta_list = np.zeros(n)
    K_list = np.zeros(n)
    v_cmd_list = np.zeros(n)
    w_cmd_list = np.zeros(n)

    # Define EKF Parameters
    # P (3x3)
    P = np.diag([0.05, 0.05, 0.05])

    # Q (3x3)
    # Increase Q if its too confident in the model and drifts
    Q = np.diag([1e-3, 1e-3, 1e-3])

    # R (2,2) "True" values would be the square of the std dev
    # Increase if values hug the gps too much
    R = np.diag([0.1, 0.1])

    # H (2x3)
    H = np.array([[1,0,0],[0,1,0]])


    ekf = EKF(x=state_ekf, P=P, Q=Q, R=R, H=H)

    for idx, i in enumerate(timelist):

        #v_cmd, w_cmd = square_command(i)
        v_cmd, w_cmd, waypt_idx = waypoint_controller(state = ekf.x, waypts = waypoint_list, waypt_idx=waypt_idx)
        
        # Update true state
        state_true = diff_drive_model(state_true, v_cmd, w_cmd, dt)

        # Simulate Noisy IMU and encoder readings
        v_enc = v_cmd + np.random.normal(0,0.1)
        w_imu = w_cmd + np.random.normal(0,0.1)
        

        # Update dead reckoning state
        state_dead_reck = diff_drive_model(state_dead_reck, v_enc, w_imu, dt)

        # Update ekf state
        # EKF predict
        ekf.predict(v=v_enc, omega=w_imu, dt=dt)

        if idx % 10 == 0:
            # Simulate noisy gps
            gps_pos = state_true[:2] + np.random.normal(0,0.1, size=2) 
            # EKF update
            ekf.update(z=gps_pos)



        # Store states for plotting
        state_true_list[idx,:] = state_true
        state_dead_reck_list[idx,:] = state_dead_reck
        state_ekf_list[idx,:] = ekf.x
        P_x_list[idx] = ekf.P[0,0]
        P_y_list[idx] = ekf.P[1,1]
        P_theta_list[idx] = ekf.P[2,2]
        v_cmd_list[idx] = v_cmd
        w_cmd_list[idx] = w_cmd
    
    # Plot

    fig, ax = plt.subplots()
    ax.plot(state_true_list[:,0], state_true_list[:,1], label='True State')
    ax.plot(state_dead_reck_list[:,0], state_dead_reck_list[:,1], label='Dead Reckoning State')
    ax.plot(state_ekf_list[:,0], state_ekf_list[:,1], label='EKF State')

    ax.set_title('EKF Simulation')
    ax.legend()

    fig2, ax2 = plt.subplots()
    ax2.plot(timelist, v_cmd_list, label='v_cmd (Linear)')
    ax2.plot(timelist, w_cmd_list, label='w_cmd (Angular)')
    ax2.set_title('Control Inputs')
    ax2.set_xlabel('Time (s)')
    #ax2.grid(True)
    ax2.legend()

    plt.show()


    return


if __name__ == "__main__":
    main()