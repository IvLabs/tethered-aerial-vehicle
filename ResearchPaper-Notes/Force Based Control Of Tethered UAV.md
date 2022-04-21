# Force Based Control Of Tethered UAV

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


$m_h \dot v_h = -g_h - B_h v_h + f_c + f_g$
$g_h = mg_h z_w$


$m_h$ is equal to apparent mass
$B_h$ is equal to damping matrix
$f_c$ is equal to the cable force applied to the human at $O_h$



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



In our case, the human is not aware of the desired path. It blindly follows the external force applied by the robot through the cable.
> The controller is designed so as to track any $C^2$ trajectory independent from external disturbances.

The close loop translational dynamics of the robot subject to the position controller is given as equation number 2.

$\dot v_R = u_R$


where $u_R$ is the virtual input.

According to equation number 2, the platform is ==infinitely stable== with respect to the interaction forces.

The cable force produced at $O_H$ on the handle is equation number 3
$f_c = t_c(||l_c||)l_c/||l_c||$


where $t_c(||l_c||)$represents the tension and  $l_c = p_R - p_H$.

Force produced on the drone ($O_R$) is $-f_c$.
Cable is considered to be of negligible mass and inertia.

$t_c(||l_c||)$ is given by eqn 4. represented as figure 2.

$\
    t_c(||l_c||)= 
\begin{cases}
\ k_c(||l_c|| - \overline{l_c})& \text{if } ||l_c|| -  \overline{l_c} > 0\\
    0              & \text{otherwise}
\end{cases}$



![](https://i.imgur.com/cxBJq8D.png =400x200)



where $k_c$ is constant elastic coefficient.


> where tc can be any continous and differentiable monotonically increasing function.
eqn 5

$t_c(||l_c||)\geq\epsilon||l_c|| + \gamma,\hspace{2mm} if \hspace{2mm}||l_c|| - \overline{l_c}>0$


### Controller
#### Admittance Control Strategy

Admittance is mass, stiffness and damping subjected to the measured external force acting on the robot.
Robot control input uR is given by eqn 6 where mA is virtual inertia.
$M_A$ is $0.8I_3$ and $B_A$ is $2.4I_3$ where $I_3$ is a $3*3$ identity matrix.  

$u_R = M_A^{-1}(-B_Av_R - f_c + u_A)$


Virtual Inertia: As our model is based on spring mass damper system, in 3D we need to assume, 3 different masses in 3 different directions, mutually perpendicular to each other. As these masses do not exist in real time, they are considered as virtual masses connected to 3 different springs in respective directions. 


In order to implement control law 6, state of the robot $(p_R ,v_R)$ and force applied by the cable $f_c$ are required. These two can be computed with onboard sensors. We define the state vector as  ( state vector $x$)
$x = [p_H^T \hspace{2mm}v_H^T\hspace{2mm}p_R^T\hspace{2mm}v_R^T]^T$


To write the dynamics as x dot equal to f (x,uA) where 
eqn 7
![](https://i.imgur.com/We1rsld.png)


fc is computed in eqn 3

To implement the control law, we only need a proportional feedback w.r.t robots position. 
eqn 8
$u_A = K_H e_R + f^r_c$


kH is proportional gain.


Fig 3
![](https://i.imgur.com/r7YfwKK.png)


fcr is constant forcing input. (desired cable force)

Block diagram of the overall control method is shown here fig 4
![](https://i.imgur.com/ikaIe3G.png)


Considering system 7, under the control law 8 we finally get the dynamics as eqn number 11 and the control law as eqn 16.

![](https://i.imgur.com/vzFBE9v.png)

![](https://i.imgur.com/WnK2n1R.png =400x80)

Our system is output strictly passive.

- $\overline l_c = 1 [m]$, $negligible \hspace{2mm} mass (less than 10 [g])$

- $p_{d_H}(0) = [−2\hspace{2mm}− 0.5\hspace{2mm}0]^T$

- $p_{d_H}(1) = [2\hspace{2mm} 0 \hspace{2mm}0]^T$