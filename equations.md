Certainly. I'll compile all the key dynamics and kinematics equations from the paper for your report. Here are the main equations:

1. Kinematics Model (Cartesian coordinates):

$$
\dot{x} = v \cos \theta
$$
$$
\dot{y} = v \sin \theta
$$
$$
\dot{\theta} = \omega
$$

Where $$q = [x, y, \theta]^T$$ is the state vector, and $$u = [v, \omega]^T$$ are linear and angular velocities.

2. Compact Form of Kinematics:

$$
\dot{q} = J(q)u
$$

Where:

$$
J(q) = \begin{bmatrix}
\cos \theta & 0 \\
\sin \theta & 0 \\
0 & 1
\end{bmatrix}
$$

3. Dynamics Model:

$$
M(q)\ddot{q} + V_m(q,\dot{q})\dot{q} + G(q) + F(\dot{q}) + \tau_d = B(q)\tau - A^T(q)\lambda
$$

Where:

$$
M(q) = \begin{bmatrix}
m & 0 & 0 \\
0 & m & 0 \\
0 & 0 & I
\end{bmatrix}
$$

$$
A^T(q) = \begin{bmatrix}
-\sin \theta \\
\cos \theta \\
0
\end{bmatrix}
$$

$$
B(q) = \frac{1}{r}\begin{bmatrix}
\cos \theta & \cos \theta \\
\sin \theta & \sin \theta \\
L & -L
\end{bmatrix}
$$

$$
\tau = \begin{bmatrix}
\tau_l \\
\tau_r
\end{bmatrix}
$$

$$
V_m(q,\dot{q}) = 0
$$

$$
\lambda = -m(\dot{x}\cos \theta + \dot{y}\sin \theta)\dot{\theta}
$$

4. Simplified Dynamics in Terms of Internal Velocities:

$$
\bar{M}(q)\dot{u} + \bar{V}_m(q,\dot{q})u + \bar{F}(\dot{q}) + \bar{\tau}_d = \bar{B}(q)\tau
$$

Where:

$$
\bar{M}(q) = \begin{bmatrix}
m & 0 \\
0 & I
\end{bmatrix}
$$

$$
\bar{B} = \frac{1}{r}\begin{bmatrix}
1 & 1 \\
L & -L
\end{bmatrix}
$$

5. Further Simplified Dynamics:

$$
\bar{M}\dot{u} + \bar{F} + \bar{\tau}_d = \bar{\tau}
$$

Where $$\bar{\tau} = [\tau_1, \tau_2]^T$$ represents linear and angular torques.

6. Kinematics in Polar Coordinates:

$$
\dot{\mathbf{x}} = \begin{bmatrix}
\dot{e} \\
\dot{\phi} \\
\dot{\alpha}
\end{bmatrix} = \begin{bmatrix}
-v \cos \alpha \\
v \sin \alpha / e \\
v \sin \alpha / e - \omega
\end{bmatrix}
$$

Where:
- $$e$$ is the distance to the target
- $$\phi$$ is the angle between the robot's heading and the target
- $$\alpha$$ is the angle between the robot's heading and the line to the target

7. Discrete-Time Kinematics (using ZOH discretization):

$$
\mathbf{x}(k+1) = \begin{bmatrix}
e(k+1) \\
\phi(k+1) \\
\alpha(k+1)
\end{bmatrix} = \begin{bmatrix}
e(k) - v(k)\cos \alpha(k)T \\
\phi(k) + v(k)\sin \alpha(k)T / e(k) \\
\alpha(k) + v(k)\sin \alpha(k)T / e(k) - \omega(k)T
\end{bmatrix}
$$

Where $$T$$ is the sampling period.

These equations form the core of the kinematic and dynamic models for the two-wheeled robot with visual feedback, which can be used in your MPC implementation.

Citations:
[1] https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/23856993/a0ad740e-2aea-45b8-acc8-9cf6273af8e5/Vision-Based_Model_Predictive_Control_for_Steering_of_a_Nonholonomic_Mobile_Robot.pdf