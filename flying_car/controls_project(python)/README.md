
# Control of a 3D Quadrotor - Project (python)


### 1. controller.py method explanations

#### 1.1 . yaw_control(self, yaw_cmd, yaw)

Simple P controller : 
define an error in yaw, err=(yaw_cmd-yaw), and returns the proportion of that error.

#### 1.2 altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0)

Altitude is controlled by a PD controller with respect vehicle attitude and defined as : 
<br><br>
$$\bar{u}_1 = k_{p-z}(z_{t} - z_{a}) + k_{d-z}(\dot{z}_{t} - \dot{z}_{a}) + \ddot{z}_t$$
<br>
$$thrust = MASS\cdot(\bar{u}_1-g)/b^z$$ 
<br>
where $z$ is altitude, $\dot{z}$ is vertical velocity, $\ddot{z}$ is acceleration and $k_{p-z}$ and $k_{d-z}$ is PD constans. $b^z$ is the [3,3] element of the rotation matrix that represent drones attitude.

#### 1.3 body_rate_control(self, body_rate_cmd, body_rate)

Body rates determine the yaw,pitch,roll rotation speed in the body frame, and controlled by a P controller : 
<br>
$p_{bodyrate}\cdot$ (body_rate_cmd - body_rate)


#### 1.4 roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd)

The most complex PD controller that uses acceleration in world frame and produce a desired roll,pitch rates in the body frame : 
<br><br>
$$b^x_c = \frac{acceleration_x\cdot mass}{-thrust}$$

$$b^y_c = \frac{acceleration_y\cdot mass}{-thrust}$$
<br>
$$\dot{b}^x_c  = k_p(b^x_c - b^x_a)$$

$$\dot{b}^y_c  = k_p(b^y_c - b^y_a)$$

where $b^x_a = R_{13}$ and $b^y_a = R_{23}$, $R$ is the rotation matrix. The given values can be converted into the angular velocities into the body frame by the next matrix multiplication. 

$$
\begin{pmatrix} roll \\ pitch \\ \end{pmatrix}  = \frac{1}{R_{33}}\begin{pmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c  \end{pmatrix} 
$$

#### 1.5 lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,acceleration_ff = np.array([0.0, 0.0]))

Lateral posiotn controller is a PD controller that use desired position and velocity to produce desired acceleration : 
<br><br>
$$
\begin{align}
\ddot{x}_{\text{command}} &=  c b^x_c \\
\ddot{x}_{\text{command}} &=  k^x_p(x_t-x_a) + k_d^x(\dot{x}_t - \dot{x}_a)+ \ddot{x}_t \\
b^x_c &= \ddot{x}_{\text{command}}/c
\end{align}
$$

### 2 .Results

### Horizontal trajectory :
<img src="img/horizontal_trajectory.png" width="600">
### Horizontal error : 
<img src="img/horizontal_err.png" width="600">
### Vertical trajectory : 
<img src="img/vertical_trajectory.png" width="600">
### Vertical error : 
<img src="img/vertical_err.png" width="600">
### 3D trajectory : 
<img src="img/3d_trajectory.png" width="600">
### Console result : 
<img src="img/console_result.png" width="600">
