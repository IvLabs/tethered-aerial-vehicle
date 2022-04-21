# Human State Aware Controller for Tethered UAV

###### tags: `ResearchPaperNotes`

![](https://i.imgur.com/UnR6c2S.png)


This paper deals with physical human robot interaction with a tethered UAV.
Application to a force based human 

We assume a inertial frame $\mathcal{F}_W = \{ O_W,x_W,y_W,z_W \}$

where $O_W$ is the arbitrary origin and $\{ x_W,y_W,z_W \}$  are unit axes.
$Z_W$ is oriented in opposite direction to gravity vector.

$\mathcal{F}_H = \{ O_H,x_H,y_H,z_H \}$
We also define a body frame rigidly attached to the handle. $O_H$ is the origin of $F_H$  and $\{ x_H,y_H,z_H \}$ are unit axes.


The state of the human is then given by the position of $O_H$ and its linear velocity defined by the vectors

$$p_H = [ p_{H_x} \hspace{2mm} p_{H_y} \hspace{2mm} p_{H_z}]^T  \in \mathbb{R}^3 \hspace{2mm} and \hspace{2mm} v_H = [ v_{H_x} \hspace{2mm} v_{H_y} \hspace{2mm} v_{H_z}]^T$$



Respectively, both with respect to $F_W$(inertial frame of reference)

The human dynamics are approximated with the mass spring damper system. Impedance model has been used as a basis to develop human robot cooperative task.

We consider the human dynamics as


$m_H \dot v_H = -g_H - B_h v_H + f_c + f_g + u_H$
$g_H = mg_H z_w$


$m_h$ is equal to apparent mass
$B_h$ is equal to damping matrix
$f_c$ is equal to the cable force applied to the human at $O_h$
$u_H$ is the sum of all the forces applied by the human that generate a translational motion

$f_c = [f_{c_x}  \hspace{2mm} f_{c_y}  \hspace{2mm} f_{c_z}]^T \in \mathbb{R}^3$



$g_h$ is equal to insert eqn
$f_g$ is the ground reaction force such that it satisfies these conditions.


- $f_g^T x_W = f_g^T y_W = 0,$
- $f_g^Tz_W >0, and$
- $\dot v_H z_W = v_H^Tz_W = 0 \hspace{2mm} i.e., the \hspace{2mm} human \hspace{2mm} is \hspace{2mm} constrained\hspace{2mm} on \hspace{2mm} the \hspace{2mm} ground.$



Therefore the human is constrained on the ground.


Condition for small medium sized aerial vehicles is as follows.
inequality
$f_c^Tz_W < m_Hg$


According to equation number 2, the platform is ==infinitely stable== with respect to the interaction forces.

The cable force produced at $O_H$ on the handle

$f_c = t_c(||l_c||)l_c/||l_c||$ $\hspace{1cm}-(3)$


where $t_c(||l_c||)$ represents the tension and  $l_c = p_R - p_H$.

Force produced on the drone ($O_R$) is $-f_c$.
Cable is considered to be of negligible mass and inertia.

$t_c(||l_c||)$ is given by

$\
    t_c(||l_c||)= 
\begin{cases}
\ k_c(||l_c|| - \overline{l_c})& \text{if } ||l_c|| -  \overline{l_c} > 0\\
    0              & \text{otherwise}
\end{cases}$ $\hspace{1cm}-(4)$



![](https://i.imgur.com/cxBJq8D.png =400x200)



where $k_c$ is constant elastic coefficient.


> where $t_c$ can be any continous and differentiable monotonically increasing function.


$t_c(||l_c||)\geq\epsilon||l_c|| + \gamma,\hspace{2mm} if \hspace{2mm}||l_c|| - \overline{l_c}>0$ $\hspace{1cm}-(5)$


### Controller
#### Admittance Control Strategy

Admittance is mass, stiffness and damping subjected to the measured external force acting on the robot.
Robot control input uR is given by 
  

$\dot v_R = M_A^{-1}(-B_Av_R - f_c + u_A + \delta)$  $\hspace{1cm}- (2)$

where $M_A$ is virtual inertia.

δ ∈ R3 is a bounded variable that takes into account all the tracking errors due to model mismatches and uncertainties, input and state limits, external disturbances, and estimation errors.

Virtual Inertia: As our model is based on spring mass damper system, in 3D we need to assume, 3 different masses in 3 different directions, mutually perpendicular to each other. As these masses do not exist in real time, they are considered as virtual masses connected to 3 different springs in respective directions. 

$M_A$ is $5I$ and $B_A = b_AI$ and $B_H = b_HI$ where $I$ is an identity matrix.

In order to implement control law 6, state of the robot $(p_R ,v_R)$ and force applied by the cable $f_c$ are required. These two can be computed with onboard sensors. We define the state vector as  ( state vector $x$)
$x = [p_H^T \hspace{2mm}v_H^T\hspace{2mm}p_R^T\hspace{2mm}v_R^T]^T$


To write the dynamics as $\dot x$ equal to $f (x,u_A)$ where 

![](https://i.imgur.com/We1rsld.png) $\hspace{1cm}-(5)$


$f_c$ is computed in $(3)$ 

<!-- To implement the control law, we only need a proportional feedback w.r.t robots position.  -->

The more the human walks fast (modeled as a low damping $B_H$), the less the force they feel on the cable. Intuitively, looking at $(2)$, the problem comes from the fact that the robot wants to apply the desired force uA but trying to stay still (due to damping term). While the human walks, the two actions are opposite and the system converges to a state where the two are in equilibrium, but none of the two objectives are fully satisfied.
To avoid this problem, the robot must consider the human’s speed while performing the guiding task.

We propose to modify the control law as

$u_A = K_H e_R + f^r_c + B_Av_H$

kH is proportional gain.

We yield the new robot dynamics as, 

eqn 25
![](https://i.imgur.com/KuAsSp0.png) $\hspace{1cm}-(25)$

Therefore the final closed loop dynamics is 


![](https://i.imgur.com/pxzfRyn.png) $\hspace{1cm}-(26)$




<!-- Fig 3
![](https://i.imgur.com/r7YfwKK.png)


$f^r_c$ is constant forcing input. (desired cable force)

Block diagram of the overall control method is shown here fig 4
![](https://i.imgur.com/ikaIe3G.png)


Considering system 7, under the control law 8 we finally get the dynamics as eqn number 11 and the control law as eqn 16.

![](https://i.imgur.com/vzFBE9v.png)

![](https://i.imgur.com/WnK2n1R.png =400x80)

Our system is output strictly passive.

- $\overline l_c = 1 [m]$, $negligible \hspace{2mm} mass (less than 10 [g])$

- $p_{d_H}(0) = [−2\hspace{2mm}− 0.5\hspace{2mm}0]^T$

- $p_{d_H}(1) = [2\hspace{2mm} 0 \hspace{2mm}0]^T$ -->


For simulation:

![](https://i.imgur.com/1y5kc5L.png)
