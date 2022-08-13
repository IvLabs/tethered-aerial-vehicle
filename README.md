# Human State Aware Controller For a Tethered UAV




## Objectives
 - To aid a human in navigating from one position to another, using a tethered aerial vehicle.
 - To propose a human-state-aware controller for this system.

## Results


## Steady State Solution

Assumptions:
- Human changes his direction in accordance with the direction of cable force.
- If the difference between magnitude of position vector (from human to drone) and original length of tether is less than or equal to zero, then cable force is zero
- If the x and y components of cable force are too small, the human velocity becomes zero.
- If the magnitude of cable force becomes zero at any instant, the human stops moving.
- Values of certain constants have been changed according to our interpretation

![](https://i.imgur.com/BfPXEjB.png)

![](https://i.imgur.com/4mycHuv.png)

![](https://i.imgur.com/ZDygPXj.png)
![](https://i.imgur.com/KH0r6tf.png)
![](https://i.imgur.com/KuyTysZ.png)



**Another method proposed by the paper was**

## Steady State Solution under Constant Control Input
Assumptions:
- The cable force is assumed to be constant
- Human and drone have the same velocity and follow a constant velocity trajectory
- The product of proportional gain and error in robot position is assumed to be constant

![](https://i.imgur.com/olVXXKu.png)
![](https://i.imgur.com/vJ7eqiU.png)
![](https://i.imgur.com/rR8vUqC.png)
![](https://i.imgur.com/FbivmCN.png)
![](https://i.imgur.com/QfNpln0.png)


    
**We thought of combining these two approaches into a Hybrid approach**

## Hybrid Control
Assumptions:
- For initial 50 seconds, it is assumed that the product of proportional gain and error in robot position is large and considered to be constant. For the rest of simulation time the steady state approach assumptions are taken into account.
    

![](https://i.imgur.com/4qkIvkt.png)
![](https://i.imgur.com/5mwgdQ0.png)
![](https://i.imgur.com/mYHJpWg.png)

![](https://i.imgur.com/zshzLEk.png)
![](https://i.imgur.com/4OteWzI.png)
