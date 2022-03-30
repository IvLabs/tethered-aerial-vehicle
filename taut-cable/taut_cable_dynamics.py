
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from math import pi,sin,cos,tan

#simulation time 
tspan=np.arange(0,20,0.01)
t=2000
# Drone params
J= 0.015
mass = 2
gravity = 9.81
rho = 0.1


# Desired control objectives
# alpha_des = -pi/8
# theta_des = 5*pi/6
# r_des = 10


# Current Angles
# alpha = 0
# theta = 0


# Derivative of control objectives
r_dot=0
r_ddot = 0
alpha_dot=0
alpha_ddot = 0
theta_dot=0
theta_ddot = 0

delta_t=0.01

# PD controllers for alpha-(from equation uAlpha)
kP_alpha = 6
kD_alpha = 1

#equation 24
kP_theta = 1
kD_theta = 2
kD_r= 6
kP_r= 6

#arrays
r_arr = [] 
r_d_arr = []
theta_arr= []
theta_d_arr = []
alpha_arr = []
alpha_d_arr = []

initial_state = np.array([1,pi/15,pi/8 ,0,0,0]) 
r=initial_state[0]
theta = initial_state[1]
alpha = initial_state[2]

# Solved_state= In each it will be updated to reach the final desired state , in varad's code 
                                                                        # in simulate function it can be seen.

des_state = np.array([10,-pi/32,9*pi/10,0,0,0]) # r,theta,alpha,r_dot,theta_dot,alpha_dot
alpha_des = 9*pi/10
theta_des=-pi/32
r_des=10
solvedState=[]
solvedState_d = [r_dot,theta_dot,alpha_dot]

for i in range(t):
    
    
    u3= -( kD_r*r_dot+ kP_r*(r-r_des) )/rho 
    T_bar = mass*gravity*(cos(alpha_des)*tan(alpha_des+theta_des)-sin(alpha_des))
    uT = T_bar + mass*gravity*sin(alpha) + mass*rho*u3  
    uAlpha = mass*(2*r_dot*alpha_dot + gravity*cos(alpha)) - mass*r*(kP_alpha*(alpha - alpha_des) + kD_alpha*(alpha_dot))
    
    
    theta_C = math.atan2(uT ,uAlpha) - alpha

    u1 = uT/(sin(alpha + theta_C))

    
    theta_error= theta-theta_C
    u2 = -1*J*(kP_theta*theta_error + kD_theta*theta_dot)

    
    #dynamicsx
    r_ddot = rho*u3
    #alpha_ddot = (-(2*r_dot*alpha_dot + gravity*cos(alpha))/r) + (((cos(theta + alpha))/mass*r)*u1)
    alpha_ddot = - (2*r_dot*alpha_dot + gravity*cos(alpha))/r + ((cos(theta_C + alpha))/(mass*r))*u1
    theta_ddot = u2/J

#   Euler approach for solving dynamics

    #  for calculating r
    r_dot=r_dot+r_ddot*delta_t
    r_d_arr.append(r_dot)
    
    r= r+ r_dot*delta_t     
    r_arr.append(r)

    #  for calculating theta
    theta_dot=theta_dot+ theta_ddot*delta_t
    theta_d_arr.append(theta_dot)
    
    theta=theta+ theta_dot*delta_t
    print(theta)
    theta_arr.append(theta)
    
    # for calculating alpha
    alpha_dot=alpha_dot + alpha_ddot*delta_t
    alpha_d_arr.append(alpha_dot)

    alpha=alpha+ alpha_dot*delta_t
    alpha_arr.append(alpha)

    # row = np.array[r,theta,alpha]
    # # solvedState.append([r,theta,alpha])
    # solvedState = np.append(solvedState,[row],axis = 0)



plt.plot(tspan,theta_arr)
plt.plot()
plt.xlabel('Time')
plt.ylabel('theta')
plt.show() 

plt.plot(tspan,r_arr)
plt.plot()
plt.xlabel('Time')
plt.ylabel('r')
plt.show() 

plt.plot(tspan,alpha_arr)
plt.plot()
plt.xlabel('Time')
plt.ylabel('alpha')
plt.show() 
    
    





















'''

# Code for producing animation    
def anime():
    l = (Ixx * 12/ MASS) * (1 / 2)                  # length of drone
    l1=l+0.15
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, aspect='equal')
    
    ax.grid()
    
    line, = ax.plot([], [], '-', lw=5.5)
    lined, = ax.plot([], [], '-', lw=5.5)
    line1, = ax.plot([], [], '-', lw=3)
    lined1, = ax.plot([], [], '-', lw=3)

    def init():
        line.set_data([], [])
        lined.set_data([], [])
        line1.set_data([], [])
        lined1.set_data([], [])
        
        return lined, line, lined1, lined

    def animate(i):
        global N
        global T
        while (i < len(t)):
            y = [solvedState[i, 1] - (l / 2) * np.cos(solvedState[i, 2]),
                solvedState[i, 1] + (l / 2) * np.cos(solvedState[i, 2])]
            z = [solvedState[i, 0] - (l / 2) * np.sin(solvedState[i, 2]),
                solvedState[i, 0] + (l / 2) * np.sin(solvedState[i, 2])]

            yd = [des_state[1] - (l / 2) * np.cos(des_state[2]), des_state[1] + (l / 2) * np.cos(des_state[2])]
            zd = [des_state[0] - (l / 2) * np.sin(des_state[2]), des_state[0] + (l / 2) * np.sin(des_state[2])]

            y1 = [solvedState[i, 1] - (l1 / 2) * np.cos(solvedState[i, 2]),
                solvedState[i, 1] + (l1 / 2) * np.cos(solvedState[i, 2])]
            z1 = [solvedState[i, 0] - (l1 / 2) * np.sin(solvedState[i, 2]),
                solvedState[i, 0] + (l1 / 2) * np.sin(solvedState[i, 2])]

            yd1 = [des_state[1] - (l1 / 2) * np.cos(des_state[2]), des_state[1] + (l1 / 2) * np.cos(des_state[2])]
            zd1 = [des_state[0] - (l1 / 2) * np.sin(des_state[2]), des_state[0] + (l1 / 2) * np.sin(des_state[2])]
            
            ax.set_xlim(solvedState[i,1]-1,solvedState[i,1]+1)
            ax.set_ylim(solvedState[i,0]-1,solvedState[i,0]+1)
            
                

            line.set_data(y, z)
            lined.set_data(yd, zd)
            line1.set_data(y1, z1)
            lined1.set_data(yd1, zd1)
            plt.draw()

            return lined1, line1, lined, line
    
    

    ani = animation.FuncAnimation(fig, animate,
                                  interval = 1, blit=False, init_func=init, repeat = False, frames=len(t))
    #ani.save('2D Min Accel.mp4',writer='ffmpeg',fps=30,bitrate=1800)

    plt.show()
    return None

simulation_time = 15 # seconds
plan_time = 5 #time for which the path has to be planned
time_points = simulation_time * 100 + 1
plan_time_points = plan_time * 100 + 1
t = np.linspace(0,simulation_time,time_points)
plan_t = np.linspace(0,plan_time,plan_time_points)


anime()

'''