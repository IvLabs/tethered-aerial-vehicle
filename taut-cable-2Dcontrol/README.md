# 2D Taut Cable Control Of Tethered UAV

Implementation of the paper [Taut Cable Control of a Tethered UAV](https://folk.ntnu.no/skoge/prost/proceedings/ifac2014/media/files/2581.pdf) by Marco Nicotra et al.

This paper looks into the case where the actuated winch controls the length of the rope while the UAV is in charge of keeping it taut. 

![](https://i.imgur.com/aq3krNg.png)



## Assumptions
 - The string is massless, inextensible and attached to the centre of mass of the UAV
 - The cable is taut throughout the process


<img src="https://render.githubusercontent.com/render/math?math={\color{white}T(t) > 0 \hspace{0.5cm} \forall t \geq0}">
<img src="https://render.githubusercontent.com/render/math?math={\color{white}T(t) = T(r(t),\alpha(t),\theta(t) \hspace{0.3cm}is}">
<img src="https://render.githubusercontent.com/render/math?math={\color{white}T = mr\dot{\alpha}^{2} -mgsin\alpha + sin(\alpha + \theta)u_1 - m\ddot{r}}">

<!-- $T(t) > 0 \hspace{0.5cm} \forall t \geq0$
$T(t) = T(r(t),\alpha(t),\theta(t) \hspace{0.3cm}is$
$T = mr\dot{\alpha}^{2} -mgsin\alpha + sin(\alpha + \theta)u_1 - m\ddot{r}$ -->



## Dynamics
Under these two assumptions, dynamics of UAV can be formulated as:

![](https://i.imgur.com/U3FFRdW.png)
## Control Objectives

![](https://i.imgur.com/avk2Zdp.png)
## Control Architecture

![](https://i.imgur.com/Zme3Geb.png)
## Results 
![](https://i.imgur.com/K3RbinH.png)
![](https://i.imgur.com/pLg130m.png)
![](https://i.imgur.com/GX4PoZT.png)

**Trajectory of UAV**

![](https://i.imgur.com/vNRT51F.png)

**Simulation**

![](https://i.imgur.com/Gx5SN9A.gif)
