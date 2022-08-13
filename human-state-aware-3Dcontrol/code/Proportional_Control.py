import numpy as np
from math import sin,cos,pi,sqrt
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def euler_dynamics(x,v,a,del_t):
    v=v+a*del_t
    x=x+v*del_t
    return v,x

del_t= 0.01
time_span = np.linspace(0,150,15000)  

#human params and position

m_H = 60
x_H = 0
y_H = 0
z_H = 1

a_H = 0                                                                     # human acceleration

v_H = 0.0001  



# forces on the system
g = 9.8
g_H = np.array([[0],[0],[m_H*g]])


k_c= 10                                                                     # spring constant
k_H = 15

I = np.identity(3)
M_A = 20*I                                                                  # virtual intertia
lc_bar = 1.5                                                                # length of tether cable
b_A = 0
B_A = b_A*I                                                                 # Drone's damping matrix
b_H = 0
B_H = b_H*I                                                                 # Human's damping matrix

#robot params and position

x_R = 1.5*cos(45*pi/180)
y_R = 0
z_R = z_H + (1.5*sin(45*pi/180))
a_R = 0                                                                     # drone's acceleration

#desired drone position

x_R_r = 2
y_R_r = 3
z_R_r = 2.5


v_R = np.array([[0],[0],[0]])                                               # velocity of robot


                                                                      
K_H = np.diag(np.array([k_H,k_H,0]))                                        # proportional gain matrix

f_cr=np.array([[0],[0],[1]])                                                # desired cable force - from calculation ()

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
a_H_arr =[]
v_H_arr = []
b_H_arr = []

for i in range(15000):
    
    v_H_arr.append(np.linalg.norm(v_H))

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
    
    if(lc_norm - lc_bar <=0):
        f_c = np.zeros((3,1))
        # v_H = np.zeros((3,1))
    else:
        f_c = k_c*(l_c - lc_bar_components)                                    # equation of cable force
    

    f_c_mod=sqrt((f_c[0,0])**2+(f_c[1,0])**2+(f_c[2,0])**2)                    # magnitude of cable force
    
    f_c_x_arr.append(f_c[0,0])
    f_c_y_arr.append(f_c[1,0])
    f_c_z_arr.append(f_c[2,0])
    f_c_arr.append(f_c_mod)


    

    # multiplying human velocity with direction

    R = p_R-p_H
    N = np.array([[0],[0],[1]])
    dot=np.dot(np.transpose(R),N)
    pro =dot[0,0]*N 
    C = R - pro

    

    
    if abs(f_c[0,0])<0.1 and abs(f_c[1,0])<0.1:
        v_H=np.array([[0],[0],[0]])


                                    
    p_Rr=np.array([[x_R_r],[y_R_r],[z_R_r]])                                  # desired drone position
    e_R = p_Rr - p_R
    u_A = np.dot(K_H,e_R)+ f_cr                                               # its an additional input that will be used to get desired control goal.(modified control law)

    f_g = np.array([[0],[0],[m_H*g - f_c[2,0]]])                              # ground reaction force
    
    if (np.linalg.norm(v_H)) != 0:
        b_H = f_c_mod/(np.linalg.norm(v_H))
    
    b_H_arr.append(b_H)
    B_H = b_H*I
    b_A = 3.5*b_H
    B_A = b_A*I
    
    
    
    a_H = - g_H - np.dot(B_H,v_H)+f_c+f_g                                     # Governing dynamical equation
    a_H_arr.append(np.linalg.norm(a_H))
    a_R = np.dot(np.linalg.inv(M_A),(np.dot(B_A,(v_H-v_R))-f_c+u_A))
    


    
    # finding velocity and position through eular dynamics

    if abs(f_c[0,0])<0.1 and abs(f_c[1,0])<0.1 and abs(f_c[2,0]>0):
        f_c = np.zeros((3,1))

    if abs(f_c[0,0])<0.1 and abs(f_c[1,0])<0.1:
        v_H=np.array([[0],[0],[0]])

    if f_c_mod == 0:
        v_H = np.zeros((3,1))
    else:
         v_H=(np.linalg.norm(v_H))*C/np.linalg.norm(C)


    # adding disturbance at any random time 

    # if i in range(3000,3005):
    #     v_H=np.array([[0.707],[0.707],[0]])
    # else:
        # v_H=(np.linalg.norm(v_H))*C/np.linalg.norm(C)

    v_R,p_R=euler_dynamics(p_R,v_R, a_R, del_t)

    v_H,p_H=euler_dynamics(p_H,v_H, a_H, del_t)
    
    

#Plotting the graphs 


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

# cable force

plot3=plt.figure(3)
plt.plot(time_span,f_c_arr)
plt.ylabel('f_c_mod')
plt.xlabel('time') 
plt.title('Cable Force')


# # 3D plot of trajectories

# # Robot trajectory

fig1 = plt.figure()

ax1 = plt.axes(projection ='3d')
ax1.plot3D(x_R_arr, y_R_arr,z_R_arr ,'green')
ax1.set_xlabel('$X$')
ax1.set_ylabel('$Y$')
ax1.set_zlabel('$Z$')
ax1.set_title('3D trajectory plot for drone')


# # Human trajectory

fig2 = plt.figure()

ax2 = plt.axes(projection ='3d')
ax2.plot3D(x_H_arr, y_H_arr,z_H_arr ,'green')
ax2.set_xlabel('$X$')
ax2.set_ylabel('$Y$')
ax2.set_zlabel('$Z$')
ax2.set_title('3D trajectory plot for human')
plt.show()




