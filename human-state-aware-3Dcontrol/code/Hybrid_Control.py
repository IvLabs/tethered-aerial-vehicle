import numpy as np
from math import sin,cos,pi,sqrt,atan2,radians
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import random

def euler_dynamics(x,v,a,del_t):
    v=v+a*del_t
    x=x+v*del_t
    return v,x
    
del_t= 0.01
time_span = np.linspace(0,70,7000)

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

x_R_r = 4  + 1.5/sqrt(2)
y_R_r = 4  + 1.5/sqrt(2)
z_R_r = 2.5


v_R = np.array([[0],[0],[0]])                                             # velocity of robot

k_H = 20
                                                                      
K_H = np.diag(np.array([k_H,k_H,0]))                                      # proportional gain matrix

f_cr=np.array([[0],[0],[0]])                                            # desired cable force - from calculation ()

# human and robot position vectors

p_H = np.array([[x_H],[y_H],[z_H]])
p_R=np.array([[x_R],[y_R],[z_R]])
p_R_r=np.array([[x_R_r],[y_R_r],[z_R_r]])



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

o = 0
p = 0

for i in range(7000):

    gamma = np.array([[3*cos(pi/4)],[3*sin(pi/4)],[0]])
    e_R = p_R_r - p_R

    x_H_arr.append(p_H[0,0])
    y_H_arr.append(p_H[1,0])
    z_H_arr.append(p_H[2,0])

    x_R_arr.append(p_R[0,0])
    y_R_arr.append(p_R[1,0])
    z_R_arr.append(p_R[2,0])
    e_R = p_R_r - p_R
    l_c = p_R - p_H 
    
    # to calculate fc via eqn 3 and 4
    
    lc_norm = np.linalg.norm(l_c)
    lc_bar_components=(lc_bar/lc_norm)*l_c 
    
    length.append(lc_norm - lc_bar)

    if i>5000:
    
    # to calculate fc via eqn 3 and 4
        if o == 0:
            print("proprtional at i= ",i)
            o= o+1

        if(lc_norm - lc_bar <=0):
           f_c = np.zeros((3,1))
        # v_H = np.zeros((3,1))
        else:
           f_c = k_c*(l_c - lc_bar_components)                                   # equation of cable force
    

        f_c_mod=sqrt((f_c[0,0])**2+(f_c[1,0])**2+(f_c[2,0])**2)               # magnitude of cable force
    
        f_c_x_arr.append(f_c[0,0])
        f_c_y_arr.append(f_c[1,0])
        f_c_z_arr.append(f_c[2,0])
        # f_c_arr.append(f_c_mod)

    # multiplying human velocity with direction

        R = p_R-p_H
        N = np.array([[0],[0],[1]])
        dot=np.dot(np.transpose(R),N)
        pro =dot[0,0]*N 
        C = R - pro
    
        if f_c_mod == 0:
            v_H = np.zeros((3,1))
        else:
             v_H=0.1*C/np.linalg.norm(C)

        p_Rr=np.array([[x_R_r],[y_R_r],[z_R_r]])                             # desired drone position
        e_R = p_Rr - p_R
        u_A = np.dot(K_H,e_R)+ f_cr                                          # its an additional input that will be used to get desired control goal.(modified control law)

        f_g = np.array([[0],[0],[m_H*g - f_c[2,0]]])                         # ground reaction force 
    
    
        a_H = - g_H - np.dot(B_H,v_H)+f_c+f_g                               # Governing dynamical equation
    
        a_R = np.dot(np.linalg.inv(M_A),(np.dot(B_A,(v_H-v_R))-f_c+u_A))


        v_R,p_R=euler_dynamics(p_R,v_R, a_R, del_t)
        if abs(f_c[0,0])<0.1 and abs(f_c[1,0])<0.1:
            v_H=np.array([[0],[0],[0]])
        p_H= p_H + v_H*del_t
    

    else:

        if p == 0:
            print("Constant control input at i = ",i)
            p=p+1
        f_c=gamma
        v_H=np.dot(np.linalg.inv(B_H),gamma)
        if i == random.randint(0,5000):
    #        print(v_H)
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


# Human 3D trajectory

fig2 = plt.figure()

ax2 = plt.axes(projection ='3d')
ax2.plot3D(x_H_arr, y_H_arr,z_H_arr ,'green')
ax2.set_xlabel('$X$')
ax2.set_ylabel('$Y$')
ax2.set_zlabel('$Z$')
ax2.set_title('3D trajectory plot for human')



# Robot position subplot

plot_3=plt.figure(3)
plot_3.suptitle('Robot Position')
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


# Robot 3D trajectory

fig4 = plt.figure()

ax1 = plt.axes(projection ='3d')
ax1.plot3D(x_R_arr, y_R_arr,z_R_arr ,'green')
ax1.set_xlabel('$X$')
ax1.set_ylabel('$Y$')
ax1.set_zlabel('$Z$')
ax1.set_title('3D trajectory plot for drone')


# cable force
plot5=plt.figure(5)
plt.plot(time_span,f_c_arr)
plt.xlabel('time')
plt.ylabel('f_c_mod')
plt.title('Cable force')

plt.show()

