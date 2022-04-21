#importing libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from math import pi,sin,cos,tan

def euler_dynamics(x,v,a,t):
    v=v+a*t
    x=x+v*t
    return v,x

#simulation time 
tspan=np.arange(0,8,0.01)
t=800                     # number of iterations

# Drone params
J= 0.015                  #moment of inertia of UAV
mass = 2                  #mass of UAV
gravity = 9.81
rho = 0.1                 #radius of winch

# Derivative of control objectives
r_dot=0                   
r_ddot = 0               
alpha_dot=0               
alpha_ddot = 0            
theta_dot=0               
theta_ddot = 0            

delta_t=0.01

# PD gain values for alpha-(from equation uAlpha)
kP_alpha = 4
kD_alpha = 3.8

# PD gain values for theta and r
kP_theta = 4
kD_theta = 3.5
kD_r= 6
kP_r= 6

#arrays for storing the updating values of control objectives and their derivatives
r_arr = []              
r_d_arr = []            
theta_arr= []           
theta_d_arr = [] 
alpha_arr = [] 
alpha_d_arr = [] 

# initial state
r = 1
theta = pi/15
alpha = pi/8

# desired states
r_des=2
alpha_des = 9*pi/10
theta_des=-pi/32

# resolving the components in 2d plane (polar to cartesian)
X = r_des*sin(alpha_des)
Y = r_des*cos(alpha_des)
X_dot = 0
Y_dot = 0
theta_des_dot = 0

des_state = np.array([X,Y,theta_des,X_dot,Y_dot,theta_des_dot]) 

solvedState=[]                                  # initialising array for storing r, theta, alpha
solvedState_d = [r_dot,theta_dot,alpha_dot]

# just for animation
y=[]
x=[]

y_dot=[]
x_dot=[]

for i in range(t):
    
    # Required Equations

    u3= -( kD_r*r_dot+ kP_r*(r-r_des) )/rho 
    T_bar = mass*gravity*(cos(alpha_des)*tan(alpha_des+theta_des)-sin(alpha_des))
    uT = T_bar + mass*gravity*sin(alpha) + mass*rho*u3  
    uAlpha = mass*(2*r_dot*alpha_dot + gravity*cos(alpha)) - mass*r*(kP_alpha*(alpha - alpha_des) + kD_alpha*(alpha_dot))
    theta_C = math.atan2(uT ,uAlpha) - alpha
    u1 = uT/(sin(alpha + theta_C))
    theta_error= theta-theta_C
    u2 = -1*J*(kP_theta*theta_error + kD_theta*theta_dot)

    #dynamics
    r_ddot = rho*u3
    alpha_ddot = - (2*r_dot*alpha_dot + gravity*cos(alpha))/r + ((cos(theta_C + alpha))/(mass*r))*u1
    theta_ddot = u2/J

   #Using Euler's method to update the values

    #  for calculating r
    r_dot,r = euler_dynamics(r,r_dot,r_ddot,delta_t)
    r_d_arr.append(r_dot)    
    r_arr.append(r)

    #  for calculating theta
    theta_dot,theta=euler_dynamics(theta,theta_dot,theta_ddot,delta_t)
    theta_d_arr.append(theta_dot)
    theta_arr.append(theta)
    
    # for calculating alpha
    alpha_dot,alpha=euler_dynamics(alpha,alpha_dot,alpha_ddot,delta_t)
    alpha_d_arr.append(alpha_dot)
    alpha_arr.append(alpha)

    # converting to cartesian coordinate
    y.append(r*sin(alpha))
    x.append(r*cos(alpha))
    y_dot.append(r*cos(alpha)*alpha_dot+ sin(alpha)*r_dot)
    x_dot.append(-r*sin(alpha)*alpha_dot + cos(alpha)*r_dot)

# print(x[0],y[0])
solvedState = np.array([y,x,theta_arr,y_dot,x_dot,theta_d_arr])
solvedState = np.transpose(solvedState)
# print(solvedState.shape)


#Plotting the graphs 
plot1=plt.figure(1)
plt.plot(tspan,theta_arr)
plt.xlabel('Time')
plt.ylabel('theta')

plot2=plt.figure(2)
plt.plot(tspan,r_arr)
plt.xlabel('Time')
plt.ylabel('r')

plot3=plt.figure(3)
plt.plot(tspan,alpha_arr)
plt.xlabel('Time')
plt.ylabel('alpha')
 
plot4=plt.figure(4)
plt.plot(x,y)
plt.xlabel('x')
plt.ylabel('y')
plt.show() 
    
# Animation code 
def anime():
    l =   ( J * 12/ mass) * (1 / 2)                  # length of drone
    l1=l+0.15                                        
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, aspect='equal')
    
    ax.grid()
    
    line, = ax.plot([], [], '-', lw=5.5)            # main body 
    lined, = ax.plot([], [], '-', lw=5.5)           # desired position
    line1, = ax.plot([], [], '-', lw=3)             #span length
    lined1, = ax.plot([], [], '-', lw=3)
    cable, = ax.plot([], [], '-', lw=1)

    def init():

        line.set_data([], [])
        lined.set_data([], [])
        line1.set_data([], [])
        lined1.set_data([], [])
        cable.set_data([],[])
        
        return lined, line, lined1, lined, cable

    def animate(i):
        global N
        global T

        while (i < len(tspan)):
            
            # calculating the position using the current state matrix
            y = [solvedState[i, 1] - (l / 2) * np.cos(-solvedState[i, 2]),
                solvedState[i, 1] + (l / 2) * np.cos(-solvedState[i, 2])]
            z = [solvedState[i, 0] - (l / 2) * np.sin(-solvedState[i, 2]),
                solvedState[i, 0] + (l / 2) * np.sin(-solvedState[i, 2])]

            # desired state
            yd = [des_state[1] - (l / 2) * np.cos(-des_state[2]), des_state[1] + (l / 2) * np.cos(-des_state[2])]
            zd = [des_state[0] - (l / 2) * np.sin(-des_state[2]), des_state[0] + (l / 2) * np.sin(-des_state[2])]

            
            y1 = [solvedState[i, 1] - (l1 / 2) * np.cos(-solvedState[i, 2]),
                solvedState[i, 1] + (l1 / 2) * np.cos(-solvedState[i, 2])]
            z1 = [solvedState[i, 0] - (l1 / 2) * np.sin(-solvedState[i, 2]),
                solvedState[i, 0] + (l1 / 2) * np.sin(-solvedState[i, 2])]

            yd1 = [des_state[1] - (l1 / 2) * np.cos(-des_state[2]), des_state[1] + (l1 / 2) * np.cos(-des_state[2])]
            zd1 = [des_state[0] - (l1 / 2) * np.sin(-des_state[2]), des_state[0] + (l1 / 2) * np.sin(-des_state[2])]
        
            
            ax.set_xlim(solvedState[i,1]-1,solvedState[i,1]+1)
            ax.set_ylim(solvedState[i,0]-1,solvedState[i,0]+1)

            
            line.set_data(y, z)
            lined.set_data(yd, zd)
            line1.set_data(y1, z1)
            lined1.set_data(yd1, zd1)
            cable.set_data([0,solvedState[i,1]],[0,solvedState[i,0]])
            plt.draw()

            return lined1, line1, lined, line, cable
    
    

    ani = animation.FuncAnimation(fig, animate,
                                  interval = 1, blit=False, init_func=init, repeat = False, frames=t)
    # ani.save('2D Min.mp4',writer='ffmpeg',fps=30,bitrate=1800)     # to save the clip

    plt.show()
    return None

simulation_time = 10 # seconds

anime()

