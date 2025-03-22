import rospy
import os 
import time
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams['font.family'] = ['FreeSans', 'Helvetica', 'Arial']
plt.rcParams['font.size'] = 14


class Estimator:
    """A base class to represent an estimator.

    This module contains the basic elements of an estimator, on which the
    subsequent DeadReckoning, Kalman Filter, and Extended Kalman Filter classes
    will be based on. A plotting function is provided to visualize the
    estimation results in real time.

    Attributes:
    ----------
        d : float
            Half of the track width (m) of TurtleBot3 Burger.
        r : float
            Wheel radius (m) of the TurtleBot3 Burger.
        u : list
            A list of system inputs, where, for the ith data point u[i],
            u[i][0] is timestamp (s),
            u[i][1] is left wheel rotational speed (rad/s), and
            u[i][2] is right wheel rotational speed (rad/s).
        x : list
            A list of system states, where, for the ith data point x[i],
            x[i][0] is timestamp (s),
            x[i][1] is bearing (rad),
            x[i][2] is translational position in x (m),
            x[i][3] is translational position in y (m),
            x[i][4] is left wheel rotational position (rad), and
            x[i][5] is right wheel rotational position (rad).
        y : list
            A list of system outputs, where, for the ith data point y[i],
            y[i][0] is timestamp (s),
            y[i][1] is translational position in x (m) when freeze_bearing:=true,
            y[i][1] is distance to the landmark (m) when freeze_bearing:=false,
            y[i][2] is translational position in y (m) when freeze_bearing:=true, and
            y[i][2] is relative bearing (rad) w.r.t. the landmark when
            freeze_bearing:=false.
        x_hat : list
            A list of estimated system states. It should follow the same format
            as x.
        dt : float
            Update frequency of the estimator.
        fig : Figure
            matplotlib Figure for real-time plotting.
        axd : dict
            A dictionary of matplotlib Axis for real-time plotting.
        ln* : Line
            matplotlib Line object for ground truth states.
        ln_*_hat : Line
            matplotlib Line object for estimated states.
        canvas_title : str
            Title of the real-time plot, which is chosen to be estimator type.
        sub_u : rospy.Subscriber
            ROS subscriber for system inputs.
        sub_x : rospy.Subscriber
            ROS subscriber for system states.
        sub_y : rospy.Subscriber
            ROS subscriber for system outputs.
        tmr_update : rospy.Timer
            ROS Timer for periodically invoking the estimator's update method.

    Notes
    ----------
        The frozen bearing is pi/4 and the landmark is positioned at (0.5, 0.5).
    """
    # noinspection PyTypeChecker
    def __init__(self):

        self.fig, self.axd = plt.subplot_mosaic(
            [['xy', 'phi'],
             ['xy', 'x'],
             ['xy', 'y'],
             ['xy', 'thl'],
             ['xy', 'thr']], figsize=(20.0, 15.0))
        
        # Initialize the true and estimated plots
        self.ln_xy, = self.axd['xy'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_xy_hat, = self.axd['xy'].plot([], 'o-c', label='Estimated')
        self.ln_phi, = self.axd['phi'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_phi_hat, = self.axd['phi'].plot([], 'o-c', label='Estimated')
        self.ln_x, = self.axd['x'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_x_hat, = self.axd['x'].plot([], 'o-c', label='Estimated')
        self.ln_y, = self.axd['y'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_y_hat, = self.axd['y'].plot([], 'o-c', label='Estimated')
        # self.ln_thl, = self.axd['thl'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_thl_hat, = self.axd['thl'].plot([], 'o-c', label='Estimated')
        # self.ln_thr, = self.axd['thr'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_thr_hat, = self.axd['thr'].plot([], 'o-c', label='Estimated')
        
        self.dt = 0.1
        
        self.canvas_title = 'N/A'

        self.d = 0.08
        self.r = 0.033
        self.u = []
        self.x = []
        self.y = []
        self.x_hat = []  # Your estimates go here!
        self.first_time = 0.0 # time.perf_counter()
        self.sub_u = rospy.Subscriber('u', Float64MultiArray, self.callback_u)
        self.sub_x = rospy.Subscriber('x', Float64MultiArray, self.callback_x)
        self.sub_y = rospy.Subscriber('y', Float64MultiArray, self.callback_y)
        self.tmr_update = rospy.Timer(rospy.Duration(self.dt), self.update)
        self.t0 = time.perf_counter()

    def callback_u(self, msg):
        self.u.append(msg.data)

    def callback_x(self, msg):
        data = list(msg.data)
        data[0] -= self.t0
        self.x.append(tuple(data))
        if len(self.x_hat) == 0:
            self.x_hat.append(tuple(data))

    def callback_y(self, msg):
        self.y.append(msg.data)


    def update(self, _):
        raise NotImplementedError

    def plot_init(self):
        self.axd['xy'].set_title(self.canvas_title)
        self.axd['xy'].set_xlabel('x (m)')
        self.axd['xy'].set_ylabel('y (m)')
        self.axd['xy'].set_aspect('equal', adjustable='box')
        self.axd['xy'].legend()
        
        self.axd['phi'].set_ylabel('phi (rad)')
        self.axd['phi'].legend()
        
        self.axd['x'].set_ylabel('x (m)')
        self.axd['x'].legend()
        
        self.axd['y'].set_ylabel('y (m)')
        self.axd['y'].legend()
        
        self.axd['thl'].set_ylabel('theta L (rad)')
        self.axd['thl'].legend()
        
        self.axd['thr'].set_ylabel('theta R (rad)')
        self.axd['thr'].set_xlabel('Time (s)')
        self.axd['thr'].legend()
        
        plt.tight_layout()

    def plot_update(self, _):
        # Update the true and estimated plots
        self.plot_xyline(self.ln_xy, self.x)
        self.plot_xyline(self.ln_xy_hat, self.x_hat)
        self.plot_philine(self.ln_phi, self.x)
        self.plot_philine(self.ln_phi_hat, self.x_hat)
        self.plot_xline(self.ln_x, self.x)
        self.plot_xline(self.ln_x_hat, self.x_hat)
        self.plot_yline(self.ln_y, self.x)
        self.plot_yline(self.ln_y_hat, self.x_hat)
        # self.plot_thlline(self.ln_thl, self.x)
        self.plot_thlline(self.ln_thl_hat, self.x_hat)
        # self.plot_thrline(self.ln_thr, self.x)
        self.plot_thrline(self.ln_thr_hat, self.x_hat)
        

    def plot_xyline(self, ln, data):
        if len(data):
            x = [d[2] for d in data]
            y = [d[3] for d in data]
            ln.set_data(x, y)
            self.resize_lim(self.axd['xy'], x, y)

    def plot_philine(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            phi = [d[1] for d in data]
            ln.set_data(t, phi)
            self.resize_lim(self.axd['phi'], t, phi)

    def plot_xline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            x = [d[2] for d in data]
            ln.set_data(t, x)
            self.resize_lim(self.axd['x'], t, x)

    def plot_yline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            y = [d[3] for d in data]
            ln.set_data(t, y)
            self.resize_lim(self.axd['y'], t, y)

    def plot_thlline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            thl = [d[4] for d in data]
            ln.set_data(t, thl)
            self.resize_lim(self.axd['thl'], t, thl)

    def plot_thrline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            thr = [d[5] for d in data]
            ln.set_data(t, thr)
            self.resize_lim(self.axd['thr'], t, thr)


    # noinspection PyMethodMayBeStatic
    def resize_lim(self, ax, x, y):
        xlim = ax.get_xlim()
        ax.set_xlim([min(min(x) * 1.05, xlim[0]), max(max(x) * 1.05, xlim[1])])
        ylim = ax.get_ylim()
        ax.set_ylim([min(min(y) * 1.05, ylim[0]), max(max(y) * 1.05, ylim[1])])


class OracleObserver(Estimator):
    """Oracle observer which has access to the true state.

    This class is intended as a bare minimum example for you to understand how
    to work with the code.

    Example
    ----------
    To run the oracle observer:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=oracle_observer \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Oracle Observer'

    def update(self, _):
        self.x_hat.append(self.x[-1])


class DeadReckoning(Estimator):
    """Dead reckoning estimator.

    Your task is to implement the update method of this class using only the
    u attribute and x0. You will need to build a model of the unicycle model
    with the parameters provided to you in the lab doc. After building the
    model, use the provided inputs to estimate system state over time.

    The method should closely predict the state evolution if the system is
    free of noise. You may use this knowledge to verify your implementation.

    Example
    ----------
    To run dead reckoning:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=dead_reckoning \
            noise_injection:=true \
            freeze_bearing:=false
    For debugging, you can simulate a noise-free unicycle model by setting
    noise_injection:=false.
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Dead Reckoning'
        self.r2d_element = self.r / (2*self.d)
        self.error = []
    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0]:

            curr_pos = self.x_hat[-1]
            xdot = np.array([[-self.r2d_element, self.r2d_element],
                            [(self.r/2) * np.cos(curr_pos[1]), (self.r/2) * np.cos(curr_pos[1])],
                            [(self.r/2) * np.sin(curr_pos[1]), (self.r/2) * np.sin(curr_pos[1])],
                            [1, 0],
                            [0, 1]])
            u = self.u[-1][1:]
            delta_x = xdot @ u * self.dt
            new_x = curr_pos[1:] + delta_x
            self.x_hat.append(np.hstack([[curr_pos[0]+self.dt], new_x]))

            self.error.append(np.abs(self.x_hat[-1] - self.x[-1]))

            print("SAVING TO FILE")
            np.savetxt('xhat1.txt', np.array(self.x_hat))


            save_to_file('/home/cc/ee106b/sp25/class/ee106b-abh/project3/xhat_dr.txt', self.x_hat)
            save_to_file('/home/cc/ee106b/sp25/class/ee106b-abh/project3/x_dr.txt', self.x)


class KalmanFilter(Estimator):
    """Kalman filter estimator.

    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    linear unicycle model at the default bearing of pi/4. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive Kalman filter update rule.

    Attributes:
    ----------
        phid : float
            Default bearing of the turtlebot fixed at pi / 4.

    Example
    ----------
    To run the Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=kalman_filter \
            noise_injection:=true \
            freeze_bearing:=true
    """
    def __init__(self):
        super().__init__()
        Qmult = rospy.get_param('Qmult', 1.0)
        Pmult = rospy.get_param('Pmult', 1.0)
        Rmult = rospy.get_param('Rmult', 1.0)
        self.canvas_title = 'Kalman Filter'
        self.phid = np.pi / 4
        self.A = np.eye(4)
        self.B = np.array([[(self.r/2) * np.cos(self.phid), (self.r/2) * np.cos(self.phid)],
                            [(self.r/2) * np.sin(self.phid), (self.r/2) * np.sin(self.phid)],
                            [1, 0],
                            [0, 1]])
        self.C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        self.Q = np.eye(4) * Qmult * 0.85
        self.P = np.eye(4) * Pmult * 100
        self.R = np.eye(2) * Rmult * 1
        self.error = []
    # noinspection DuplicatedCode
    # noinspection PyPep8Naming
    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0] and len(self.u) > 0:
            
            # Step 5: State Extrapolation
            x_hat_new = self.A @ self.x_hat[-1][2:] + (self.B @ self.u[-1][1:]) * self.dt

            # Step 6: Covariance Extrapolation
            self.P = self.A @ self.P @ self.A.T + self.Q

            # Step 7: Kalman Gain
            K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
            
            # Step 8: State Update
            x_hat_final = x_hat_new + K @ (self.y[-1][1:] - self.C @ x_hat_new)

            # Step 9: Covariance Update
            self.P = (np.eye(4) - K @ self.C) @ self.P

            self.x_hat.append(np.hstack([[self.x_hat[-1][0]+self.dt, self.phid], x_hat_final]))
            self.error.append(np.abs(self.x_hat[-1] - self.x[-1]))

            print("SAVING TO FILE")
            np.savetxt('xhat1.txt', np.array(self.x_hat))


            save_to_file('/home/cc/ee106b/sp25/class/ee106b-abh/project3/xhat.txt', self.x_hat)
            save_to_file('/home/cc/ee106b/sp25/class/ee106b-abh/project3/x.txt', self.x)

# noinspection PyPep8Naming
class ExtendedKalmanFilter(Estimator):
    """Extended Kalman filter estimator.

    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    unicycle model and linearize it at every operating point. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive extended Kalman filter update rule.

    Hint: You may want to reuse your code from DeadReckoning class and
    KalmanFilter class.

    Attributes:
    ----------
        landmark : tuple
            A tuple of the coordinates of the landmark.
            landmark[0] is the x coordinate.
            landmark[1] is the y coordinate.

    Example
    ----------
    To run the extended Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=extended_kalman_filter \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Extended Kalman Filter'
        # TODO: Your implementation goes here!
        # You may define the Q, R, and P matrices below.
        self.Q = np.eye(5) * 1
        self.R = np.eye(3) * 1
        self.R = np.array([[1, 0.5, 0.1], [0.5, 0.1, 0.2], [0.1, 0.2, 0.2]])*0.001
        self.P = np.eye(5) * 1
        self.last_t = None
        self.C = np.array([[1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 1, 0, 0]])


    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0] and len(self.u) > 0:
            # TODO: Your implementation goes here!
            # You may use self.u, self.y, and self.x[0] for estimation

            # Step 5: State Extrapolation
            x_new = self.g(self.x_hat[-1][1:], self.u[-1])
            print("PREVIOUS X_HAT: ", self.x_hat[-1][1:])
            print("X HAT: ", x_new)
            print("X ACTUAL: ", self.x[-1][1:])
            # Step 6: Dynamics Linearization
            self.A = self.approx_A(self.x_hat[-1][1:], self.u[-1])

            # Step 7: Covariance Extrapolation
            self.P = self.A @ self.P @ self.A.T + self.Q

            # Step 9: Kalman Gain
            K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)

            # Step 10: State Update
            y = np.array(self.y[-1][1:])[:, np.newaxis]
            print("X NEW @ C: ", (self.C @ x_new).shape)
            print("DIFFERENCE OF X NEW AND MEASURED: ", y - self.C @ x_new)
            x_hat_final = x_new + K @ (y - self.C @ x_new)

            # Step 11: Covariance Update
            self.P = (np.eye(5) - K @ self.C) @ self.P

            self.x_hat.append(np.hstack([[self.x_hat[-1][0]+self.dt], x_hat_final.flatten()]))
            print("LAST T: ", self.x_hat[-1][0])
            print("DT: ", self.x_hat[-1][0] - self.x_hat[-2][0])
    def g(self, x, u):
        phi_dot = (self.r / (2 * self.d) * (u[1] - u[0]))
        v = self.r / 2 * (u[0] + u[1])
        return np.array([
            [x[0] + phi_dot * self.dt],
            [x[1] + v * np.cos(x[0]) * self.dt],
            [x[2] + v * np.sin(x[0]) * self.dt],
            [x[3] + u[0] * self.dt],
            [x[4] + u[1] * self.dt]
        ])
    
    def approx_A(self, x, u):
        v = (self.r / 2) * (u[0] + u[1])
        return np.array([
            [1, 0, 0, 0, 0],
            [-v * np.sin(x[0]) * self.dt, 1, 0, 0, 0],
            [v * np.cos(x[0]) * self.dt, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])


def save_to_file(filename, data):
    """
    Save data to a file in append mode. If the file doesn't exist, create it.
    
    Parameters:
        filename (str): The name of the file to save the data to.
        data (np.ndarray): The data to save.
    """
    # Ensure the data is a NumPy array
    data = np.array(data, dtype=float)
    if os.path.exists(filename):
        os.remove(filename)
    
    # Check if the file exists
    if not os.path.exists(filename):
        print(f"File '{filename}' does not exist. Creating it.")
        # Create the file and write the header (optional)
        with open(filename, 'w') as f:
            f.write("# This file contains data in append mode.\n")
    
    # Append the data to the file
    with open(filename, 'ab') as f:  # 'ab' mode opens the file in binary append mode
        np.savetxt(f, data)

