import numpy as np
from math import sin,cos,pi,sqrt,atan2,radians
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import random

del_t= 0.01
time_span = np.linspace(0,50,5000)

m_H = 60
x_H = 0
y_H = 0
z_H = 1

a_H = 0                                                                    # human acceleration

v_H_bar = 0.1  



# forces on the system
g = 9.8
g_H = np.array([[0],[0],[m_H*g]])
k_c= 10                                                                    # spring constant


I = np.identity(3)
M_A = 5*I                                                                  # virtual intertia
lc_bar = 1.5                                                               # length of tether cable
B_A = 100*I                                                                # Drone's damping matrix
B_H = 30*I                                                                 # Human's damping matrix

#robot params and position

x_R = 1.8/sqrt(2)
y_R = 1.8/sqrt(2)
z_R = z_H 
a_R = 0                                                                    # drone's acceleration

#desired drone position

x_R_r = 3+x_R
y_R_r = 3+y_R
z_R_r = 2.5


v_R = np.array([[0],[0],[0]])                                             # velocity of robot

k_H = 100
                                                                      
K_H = np.diag(np.array([k_H,k_H,0]))                                      # proportional gain matrix


# human and robot position vectors

p_H = np.array([[x_H],[y_H],[z_H]])
p_R=np.array([[x_R],[y_R],[z_R]])



# lists for plotting

x_H_arr =[]
y_H_arr = []
z_H_arr = []

x_R_arr =[]
y_R_arr = []
z_R_arr = []

f_c_x_arr=[]
f_c_y_arr=[]
f_c_z_arr=[]

f_c_arr=[]

length=[]


for i in range(5000):

    

    x_H_arr.append(p_H[0,0])
    y_H_arr.append(p_H[1,0])
    z_H_arr.append(p_H[2,0])

    x_R_arr.append(p_R[0,0])
    y_R_arr.append(p_R[1,0])
    z_R_arr.append(p_R[2,0])
    l_c = p_R - p_H 
    
    # to calculate fc via eqn 3 and 4
    
    lc_norm = np.linalg.norm(l_c)
    lc_bar_components=(lc_bar/lc_norm)*l_c 
    
    length.append(lc_norm - lc_bar)

    
    
    if x_R_r<p_R[0,0]<x_R_r+0.3 and y_R_r<p_R[1,0]<y_R_r+0.5:
        gamma = np.zeros((3,1))
    else:
        gamma = np.array([[3*cos(pi/4)],[3*sin(pi/4)],[0]])
    f_c=gamma
    v_H=np.dot(np.linalg.inv(B_H),gamma)
    if i == random.randint(0,5000):
    #    print(v_H)
        print(lc_norm - lc_bar)
        
        
    v_R=v_H
    p_H=p_H+v_H*del_t
    p_R=p_R+v_R*del_t
    f_c_arr.append(np.linalg.norm(f_c))




# Human position subplot

plot_1=plt.figure(1)
plot_1.suptitle("Human Position")

plt.subplot(3,1,1)
plt.plot(time_span,x_H_arr)
plt.xlabel('time')
plt.ylabel('x_H')

plt.subplot(3,1,2)
plt.plot(time_span,y_H_arr)
plt.xlabel('time')
plt.ylabel('y_H')

plt.subplot(3,1,3)
plt.plot(time_span,z_H_arr)
plt.xlabel('time')
plt.ylabel('z_H')



# Robot position subplot

plot_2=plt.figure(2)
plot_2.suptitle('Robot Position')
plt.subplot(3,1,1)

plt.plot(time_span,x_R_arr)
plt.xlabel('time')
plt.ylabel('x_R')

plt.subplot(3,1,2)
plt.plot(time_span,y_R_arr)
plt.xlabel('time')
plt.ylabel('y_R')

plt.subplot(3,1,3)
plt.plot(time_span,z_R_arr)
plt.xlabel('time')
plt.ylabel('z_R')



plot3=plt.figure(3)
plt.plot(time_span,f_c_arr)
plt.ylabel('cable force')
plt.xlabel('time') 
plt.title('Cable force')

# Robot trajectory

fig4 = plt.figure()

ax1 = plt.axes(projection ='3d')
ax1.plot3D(x_R_arr, y_R_arr,z_R_arr ,'green')
ax1.set_xlabel('$X$')
ax1.set_ylabel('$Y$')
ax1.set_zlabel('$Z$')
ax1.set_title('3D trajectory plot for drone')


# Human trajectory

fig5 = plt.figure()

ax2 = plt.axes(projection ='3d')
ax2.plot3D(x_H_arr, y_H_arr,z_H_arr ,'green')
ax2.set_xlabel('$X$')
ax2.set_ylabel('$Y$')
ax2.set_zlabel('$Z$')
ax2.set_title('3D trajectory plot for human')


plt.show()






   