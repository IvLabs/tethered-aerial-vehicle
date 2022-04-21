# Taut Cable Control of a Tethered UAV

###### tags: `ResearchPaperNotes`

## Model Dynamics
There are three control inputs($u$<sub>1</sub>, $u$<sub>2</sub> & $u$<sub>3</sub>) and two actuators($f$<sub>1</sub> & $f$<sub>2</sub>).


![](https://i.imgur.com/aq3krNg.png =300x400)


$u$<sub>1</sub> = $f$<sub>1</sub>+$f$<sub>2</sub>
$u$<sub>2</sub> = ($f$<sub>1</sub>-$f$<sub>2</sub>)$b$



> Assume the string is massless, inextensible and attached to the center of mass of the UAV.
 
Total kinetic and potential energy of UAV is


![](https://i.imgur.com/hHnsDRA.png) 



The general dynamic model is as follows

![](https://i.imgur.com/zWuKSU5.png)








> The cable is assumed taut throughout the process.



![](https://i.imgur.com/VJYUcaM.png)






Under these two assumptions the dynamics of the UAV can be reformatted as:


![](https://i.imgur.com/U3FFRdW.png)



## Control Objective 


![](https://i.imgur.com/avk2Zdp.png)


## Control Architecture


![](https://i.imgur.com/Zme3Geb.png)




### Outer Loop Control
> To design outer loop control law we assume θ<sub>$c$</sub> (commanded theta) as virtual input to inner loop control.

==Three control objectives should be reduced to two independent conditions.(Lemma 6 conclusion)==

In outer loop, we control $r$ and  $\alpha$ and find the control equations to minimise the error in $r$ and $\alpha$.
The governing two equations are equations 10 and 11.

![](https://i.imgur.com/BmoXFsv.png)


Since dynamics of $u$<sub>3</sub> is independent from rest of the system, we define it's control law separately 

![](https://i.imgur.com/hD7SJdN.png)



$r$ and $\alpha$ which we get from the feedback are controlled by $u$<sub>3</sub> and $u$<sub>1</sub>

![](https://i.imgur.com/EohI2Up.png)



By substituting $u$<sub>3</sub> in eqn 18, we get component of $u$<sub>1</sub> along the cable($u_T$) and from eqn 19, we get the perpendicular component($u_\alpha$)

![](https://i.imgur.com/uvWPWq3.png)


From eqn 20, we finally get ${u_1}$.

Therefore $u_1$ and $u_3$ are reduced to single control system.
The system (4) subject to constraint (2) will be true only if the attainable configuration is 
![](https://i.imgur.com/qDS6oZQ.png)


and the tension is 
![](https://i.imgur.com/SUSOi2T.png)


### Inner Loop Control
Now the commanded theta(output of outer loop) will act as the desired theta for inner loop.
$\tilde{\theta} = θ − θ$<sub>$c$</sub>

Now substituting, $θ = θ$<sub>$c$</sub> + $\tilde{\theta}$

In eqn 11 and eqn 3 ,we get 



![](https://i.imgur.com/hGRkjeW.png =450x70)



If $\tilde{\theta}$ is not equal to zero, it could destabilise the outer loop dynamics or lead to violations of taut cable constraints. These two problems are addressed in inner loop control.

The stability of inner/outer loop are interconnected and the taut cable constraint will instead be enforced in inner loop.

The inner loop is controlled by a PD controller.
The following proposition discussed expresses the above interpretations.


 ![](https://i.imgur.com/0S88P0M.png)









​

