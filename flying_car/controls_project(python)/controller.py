"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MIN_THRUST = 2.0
MAX_TORQUE = 1.0

class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""

        omega = 8.0
        delta = 0.7

        self.z_k_p      = omega**2
        self.z_k_d      = 2.0*delta*omega

        p = .1
        d = 2.0

        # omega = 1.0
        # delta = 1.8

        self.x_k_p      = p
        self.x_k_d      = d
        self.y_k_p      = p
        self.y_k_d      = d

        self.k_p_roll   = 2.0
        self.k_p_pitch  = 2.0
        self.k_p_yaw    = 1.70

        self.k_p_p      = 25.0
        self.k_p_q      = 25.0
        self.k_p_r      = 6.0

        return    

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]
            
        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        
        
        return (position_cmd, velocity_cmd, yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame
        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command   
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """

        acc_x = self.x_k_p * (local_position_cmd[0]-local_position[0]) + \
                self.x_k_d * (local_velocity_cmd[0]-local_velocity[0]) + \
                acceleration_ff[0]

        acc_y = self.y_k_p * (local_position_cmd[1]-local_position[1]) + \
                self.y_k_d * (local_velocity_cmd[1]-local_velocity[1]) + \
                acceleration_ff[1]

        return np.array([acc_x, acc_y])
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command
        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up) 
        Returns: thrust command for the vehicle (+up)
        """
        rot_mat = euler2RM(attitude[0],attitude[1],attitude[2])

        b_z = rot_mat[2,2]

        alt_acc = self.z_k_p * (altitude_cmd-altitude) + \
                  self.z_k_d * (vertical_velocity_cmd-vertical_velocity) + \
                  acceleration_ff

        thrust = DRONE_MASS_KG * (alt_acc+GRAVITY) / b_z

        return np.clip(thrust, MIN_THRUST, MAX_THRUST)
        
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        rot_mat = euler2RM(attitude[0],attitude[1],attitude[2])

        b_x = acceleration_cmd[0]/-thrust_cmd
        b_y = acceleration_cmd[1]/-thrust_cmd

        b_x_c = rot_mat[0,2]
        b_y_c = rot_mat[1,2]

        b_xy_dot = np.array([self.k_p_roll *(b_x - b_x_c),
                             self.k_p_pitch*(b_y - b_y_c)]).T 


        mat = np.array([[rot_mat[1][0],-rot_mat[0][0]],
                        [rot_mat[1][1],-rot_mat[0][1]]])/rot_mat[2][2]

        p_q_cmd = np.matmul(mat,b_xy_dot)

        return p_q_cmd
    
    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        moments = np.array([0.0,0.0,0.0])

        moments[0] = self.k_p_p * (body_rate_cmd[0]-body_rate[0]) * MOI[0] 
        moments[1] = self.k_p_q * (body_rate_cmd[1]-body_rate[1]) * MOI[1]
        moments[2] = self.k_p_r * (body_rate_cmd[2]-body_rate[2]) * MOI[2] 

        return np.clip(moments, -MAX_TORQUE, MAX_THRUST)
    
    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """

        err = (yaw_cmd - yaw)
        if err > np.pi or err < -np.pi:
            err = -err
        r_c = self.k_p_yaw * err 
        return r_c

    
