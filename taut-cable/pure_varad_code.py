import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import matplotlib.animation as animation
import math

'''
CURRENTLY ALL DOUBLE DERIVATIVE STATES ARE NOT ZERO
Trajectory Planning implemented
Point to Point Striaght Line Trajectory Planning Done
Minimum Acceleration Path implemented
Minimum Jerk Path implemented
'''
'''
This project aims to develop a 2D quadcoptor controller.
The controller planned to control the quadcoptor model are:
1. PID
2. LQR
3. MPC (Maybe)
The 2D quadcoptor model can be modelled as:
z'' = -g + (u1/m) * cos(phi)
y'' = -(u1/m) * sin(phi)
phi'' = u2 * 1/ I_xx
The values of the drone are picked up from the Aerial Robotics Course.
'''

PLAN_TRAJ = True, 3
#Set PLAN_TRAJ to True if trajectory planning is to be implemented.
# Set no to
# 1 for Minimum Velocity
# 2 for Minimum Acceleration
# 3 for Minimum Jerk.


##--------QUADROTOR MODEL-----------###

class Quadrotor:
    
    
    def __init__(self):
        super().__init__()
        self.MASS = 0.1800
        self.GRAVITY = 9.81
        self.Ixx = 0.00025
        self.maxF = 3.5316
        self.minF = 0.0
        self.initial_state = np.array([np.random.randint(-5,5), 
                                         np.random.randint(-5,5), np.random.uniform(-2*np.pi/3,2*np.pi/3)  ,0,0,0]) 
        # self.initial_state = np.array([np.random.randint(-5,5), 
        #                                np.random.randint(-5,5), 0 ,0,0,0])   
        #z,y,phi,zdot,ydot,phidot.
    
        #the initial state vector contains all the state and their dervitaive
        #The self.initial_state wil be a 1*6 vector.
        self.current_state = self.initial_state
    
    def dynamics(self,state,t,u):
        z,y,phi,zdot,ydot,phidot = state    #Unpack the state vector
        u1 , u2 = u                         #Unpack the input vector 'u' into thrust and moment vectors
    
        
        state_derivative = [zdot, ydot, phidot, 
                            -self.GRAVITY + (u1/self.MASS) * np.cos(phi),
                            -(u1/self.MASS)*np.sin(phi), 
                            u2/self.Ixx]
                                            #Writing the derivative function as per the quadc model for odeint
        return state_derivative
    
    
    
    # def simulate(self,u,t):
    #     solvedState = self.initial_state
    #     timpepoints = len(t)
    #     for i in range(timepoints):
    #         tspan = [t[i-1],t[i]]            
    #         # temp = odeint(self.dynamics,self.current_state,t,args=(u[i],))
    #         # self.current_state = temp[1,:]
    #         self.current_state = odeint(self.dynamics,self.current_state,t,args=(u[i],))[1,:]
    #         solvedState = np.append(solvedState,self.current_state, axis=0)
    #     return solvedState
 
class TrajectoryPlan:
    
    def __init__(self):
        pass
        
    @staticmethod    
    def minVelPath(Drone,des_state,plan_t,t):
        initial_state = [Drone.initial_state[0],Drone.initial_state[1],0,0,0,0]
        #copy the inital state in the format of z,y,zdot,ydot,zddot,yddot.
        final_time = plan_t[-1]
        #take the last element of the planning time array
        
        stateMatrix = np.empty([len(t),6])
        stateMatrix[0,:] = initial_state 
        stateMatrix[len(plan_t),:] = des_state # at the planned time the final state should be the desired state.
        '''
        We will be implementing shortest path or minimum velocity path
        
        So the trajectory is defined by 
        x(t) = c_1 * t + c_0
        
        constrained to the condition x(0) is the intial state and x(T) is the final condition.
        this reduces the problem to find out the coefficients c_0 and c_1 found by algebra
        
        '''
        C_z_0 = Drone.initial_state[0]
        C_z_1 = ( des_state[0] - C_z_0 )/final_time
        
        C_y_0 = Drone.initial_state[1]
        C_y_1 = ( des_state[1] - C_y_0 )/final_time
        
        for i in range(1,len(t)):
            
            if ( i < len(plan_t) ):        
                z = C_z_1 * t[i] + C_z_0
                y = C_y_1 * t[i] + C_y_0
                
                vel_z = 0
                vel_y = 0
                
                # vel_z = C_z_1
                # vel_y = C_y_1
                
                stateMatrix[i,:] = [z,y,vel_z,vel_y,0,0]
                
            if (i > len(plan_t)):
                stateMatrix[i,:] = des_state
        
        return stateMatrix
    
    @staticmethod
    def minAccelnPath(Drone,des_state,plan_t,t):
        '''
        The trajectory for the minimum acceleration path is defined by a cubic polynomial with
        constrains on initial and final ; position and velocity.
        
        z(t) = Az * t^3 + Bz * t^2 + Cz * t + Dz
        
        and the constraints are 
        z(0) and z'(0) = initial_state
        z(T) and z'(T) = final_state
        
        
        z(0) = A * 0 + B* 0 + C* 0 + D
        z'(0) = A * 3 0 ^2 + B * 2 0 + C  + 0 D
        z(T) = A T^3 + B T^2 + C T + D        
        z'(T) = A * 3T^2 + B * 2T + C  + 0 D
        
        which can be written as 
        z(0)        0 0 0 1                     A
        z'(0)   =   0 0 1 0             *       B
        z(T)        T^3 T^2 T 1                 C
        z'(T)       3T^2 2T 1 0                 D
        the coeffiecnts can be obtained by using simple linear alegbra.
        
        Similar thing is done to the Y axis to find out the desired trajectory 
        
        '''
        
        final_time = plan_t[-1] # time for plannig.
        stateMatrix = np.empty([len(t),6])
        stateMatrix[:] = des_state
        #statematrix is the desired states at all time points which the controller will follow
        # in the format of z,y,zdot,ydot,zddot,yddot.
        
        coeffMatrix = np.array([
            [0,0,0,1],
            [0,0,1,0],
            [pow(final_time,3),pow(final_time,2),final_time,1],
            [3*pow(final_time,2),2*final_time,1,0]
        ])
                
        z_constriants = np.array([Drone.initial_state[0],Drone.initial_state[3],des_state[0],des_state[2]]) 
        #z(0),z(T),zdot(0),zdot(T)
        y_constriants = np.array([Drone.initial_state[1],Drone.initial_state[4],des_state[1],des_state[3]])
        #y(0),y(T),ydot(0),ydot(T)
        
        # z_constriants = np.array([Drone.initial_state[0],0,des_state[0],0]) #z(0),z(T),zdot(0),zdot(T)
        # y_constriants = np.array([Drone.initial_state[1],0,des_state[1],0]) #y(0),y(T),ydot(0),ydot(T)
        
        z_coeff = np.linalg.solve(coeffMatrix,z_constriants)                #solve for the coefficients
        print("The Z coeff are: ",z_coeff,"And its constraints are ", z_constriants)
        y_coeff = np.linalg.solve(coeffMatrix,y_constriants)                #solve for the coeffients
        print("The Y coeff are: ",y_coeff,"And its constraints are ", y_constriants)
        
        stateMatrix[0,:] = [Drone.initial_state[0],Drone.initial_state[1],0,0,0,0]
        
        zvel_coeff = [3*z_coeff[0],2*z_coeff[1],z_coeff[2]]
        yvel_coeff = [3*y_coeff[0],2*y_coeff[1],y_coeff[2]]
        
        stateMatrix[0:len(plan_t),0] = np.polyval(z_coeff,plan_t)
        stateMatrix[0:len(plan_t),1] = np.polyval(y_coeff,plan_t)
        stateMatrix[0:len(plan_t),2] = np.polyval(zvel_coeff,plan_t)
        stateMatrix[0:len(plan_t),3] = np.polyval(yvel_coeff,plan_t)
        stateMatrix[0:len(plan_t),4] = 0
        stateMatrix[0:len(plan_t),5] = 0            
        
        return stateMatrix
    
    @staticmethod
    def minJerkPath(Drone,des_state,plan_t,t):
        '''
        The trajectory for minimum jerk path is defined by a fifth order polyomial
        with constraints on both initial and final ; positions, velocities and acceleration
            So, 
            z(t) = A z^5 + B z^4 + C z^3 + D z^2 + E z + F
            and the boundary conditions are
            
            z(0),z'(0),z''(0) = initial state 
            z(T),Z'(T),z''(T) = final state
            
            The coeficients are found out by simple linear algebra
            
        '''
        T = plan_t[-1]
        stateMatrix = np.empty([len(t),6])
        stateMatrix[:] = des_state
        #statematrix is the desired states at all time points which the controller will follow
        # in the format of z,y,zdot,ydot,zddot,yddot.
        
        coeffMatrix = np.array([
            [0,0,0,0,0,1],
            [pow(T,5),pow(T,4),pow(T,3),pow(T,2),T,1],
            [0,0,0,0,1,0],
            [5*pow(T,4),4*pow(T,3),3*pow(T,2),2*T,1,0],
            [0,0,0,2,0,0],
            [20*pow(T,3),12*pow(T,2),6*T,2,0,0]
        ])
        z_constriants = np.array([Drone.initial_state[0],des_state[0],    Drone.initial_state[3],des_state[2],0,0])
        #z(0),z(T),zdot(0),zdot(T),zddot(0),zddot(T)        
        y_constriants = np.array([Drone.initial_state[1],des_state[1],    Drone.initial_state[4],des_state[3],0,0])
        #y(0),y(T),ydot(0),ydot(T),yddot(0),yddot(T)
        stateMatrix[0,:] = [Drone.initial_state[0],Drone.initial_state[1],0,0,0,0]
        z_coeff = np.linalg.solve(coeffMatrix,z_constriants)
        y_coeff = np.linalg.solve(coeffMatrix,y_constriants)
        
        zvel_coeff = [5*z_coeff[0] , 4*z_coeff[1],3*z_coeff[2],2*z_coeff[3],z_coeff[4]]
        yvel_coeff = [5*y_coeff[0] , 4*y_coeff[1],3*y_coeff[2],2*y_coeff[3],y_coeff[4]]
        zaccel_coeff = [20*z_coeff[0],12*z_coeff[1],6*z_coeff[2],2*z_coeff[3]]
        yaccel_coeff = [20*y_coeff[0],12*y_coeff[1],6*y_coeff[2],2*y_coeff[3]]
        
        stateMatrix[0:len(plan_t),0] = np.polyval(z_coeff,plan_t)
        stateMatrix[0:len(plan_t),1] = np.polyval(y_coeff,plan_t)
        stateMatrix[0:len(plan_t),2] = np.polyval(zvel_coeff,plan_t)
        stateMatrix[0:len(plan_t),3] = np.polyval(yvel_coeff,plan_t)
        stateMatrix[0:len(plan_t),4] = np.polyval(zaccel_coeff,plan_t)
        stateMatrix[0:len(plan_t),5] = np.polyval(yaccel_coeff,plan_t)
        
        return stateMatrix
        
    
def simulate(TUNING_MATRIX,des_stateMatrix,t):
    solvedState = Drone.initial_state
    timepoints = len(t)
    Kp_z, Kd_z , Ki_z = TUNING_MATRIX[0,:]
    Kp_y, Kd_y , Ki_y = TUNING_MATRIX[1,:]
    Kp_th, Kd_th , Ki_th = TUNING_MATRIX[2,:]
    
    for i in range(1,timepoints):
        
        des_state = des_stateMatrix[i,:]
        
        # u1 = Drone.MASS * ( Drone.GRAVITY +
        #                     Kd_z * (des_state[2] - Drone.current_state[3]) +
        #                     Kp_z * (des_state[0] - Drone.current_state[0])
        #                     )
            
        # phi_c = (-1/Drone.GRAVITY) * (Kd_y * (des_state[3] - Drone.current_state[4]) +
        #                               Kp_y * (des_state[1] - Drone.current_state[1]))
            
        # u2 = Drone.Ixx * (Kd_th * (-1 * Drone.current_state[5]) +
        #                   Kp_th * (phi_c - Drone.current_state[2]) )
        
        u1 = Drone.MASS * ( Drone.GRAVITY + des_state[4] + 
                            Kd_z * (des_state[2] - Drone.current_state[3]) +
                            Kp_z * (des_state[0] - Drone.current_state[0])
                            )
            
        phi_c = (-1/Drone.GRAVITY) * ( des_state[5] +
                                    Kd_y * (des_state[3] - Drone.current_state[4]) +
                                    Kp_y * (des_state[1] - Drone.current_state[1]))
            
        u2 = Drone.Ixx * (Kd_th * (-1 * Drone.current_state[5]) +
                          Kp_th * (phi_c - Drone.current_state[2]) )
        
        if u1 > Drone.maxF:
            u1 = Drone.maxF
        if u1 < 0:
            u1 = 0

        u = [u1,u2]
                 
        # temp = odeint(self.dynamics,self.current_state,t,args=(u[i],))
        # self.current_state = temp[1,:]
        Drone.current_state = odeint(Drone.dynamics,Drone.current_state,t,args=(u,))[1,:]
        #solvedState = np.append(solvedState,Drone.current_state, axis=0)
        solvedState = np.vstack((solvedState,Drone.current_state))
        
        
        
    return solvedState

def plotResults(solvedState,des_stateMatrix,t):
        
    fig = plt.figure()
    spec = fig.add_gridspec(3,4)
    
    ax1 = fig.add_subplot(spec[0:3,0:3])
    ax1.set_title('Z-Y Plane Trajectory')
    ax1.grid(True)
    ax1.plot(solvedState[:,1],solvedState[:,0],'k',label = 'Trajectory')
    ax1.plot(des_stateMatrix[:,1],des_stateMatrix[:,0],'m:',label = 'Desired Trajectory')
    ax1.legend()

        
    ax2 = fig.add_subplot(spec[0,3])
    ax2.set_title('Z Trajectory')
    ax2.grid(True)
    ax2.plot(t,solvedState[:,0],'r',label = 'Z-axis')
    ax2.plot(t,solvedState[:,3],'r:',label = 'Z-velocity')
    #ax2.plot(t,des_stateMatrix[:,0],'m:',label = 'Desired Z Trajectory')
    
    
    ax3 = fig.add_subplot(spec[1,3])
    ax3.set_title('Y Trajectory')
    ax3.grid(True)
    ax3.plot(t,solvedState[:,1],'g',label = 'Y-axis')
    ax3.plot(t,solvedState[:,4],'g:',label = 'Y-velocity')
    #ax3.plot(t,des_stateMatrix[:,1],'m:',label = 'Desired Y Trajectory')
    
    ax4 = fig.add_subplot(spec[2,3])
    ax4.set_title('Phi Trajectory')
    ax4.grid(True)
    ax4.plot(t,solvedState[:,2],'b',label = 'Phi')
    plt.show()
    
    
    return None

def anime():
    l = (Drone.Ixx * 12/ Drone.MASS) * (1 / 2)                  # length of drone
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


# Simulation Time Parameters    
simulation_time = 15 # seconds
plan_time = 5 #time for which the path has to be planned
time_points = simulation_time * 100 + 1
plan_time_points = plan_time * 100 + 1
t = np.linspace(0,simulation_time,time_points)
plan_t = np.linspace(0,plan_time,plan_time_points)


Drone = Quadrotor()
des_state = np.array([np.random.randint(-5,5),np.random.randint(-5,5),0,0,0,0]) # z,y,zdot,ydot,zdotdot,y dotdot
print("Initial State:",Drone.initial_state)
print("Final State:",des_state)

TUNING_MATRIX = np.array([
    [5,4,0],
    [3,3,0],
    [60,10,0]    
])

# TUNING_MATRIX = np.array([
#     [80,20,0],
#     [25,5,0],
#     [100,10,0]    
# ])

if PLAN_TRAJ[0] == False:
    des_stateMatrix = np.empty([len(t),6])
    des_stateMatrix[:] = des_state

if PLAN_TRAJ[0] == True and PLAN_TRAJ[1] == 1:
    des_stateMatrix = TrajectoryPlan.minVelPath(Drone,des_state,plan_t,t)

if PLAN_TRAJ[0] == True and PLAN_TRAJ[1] == 2:
    des_stateMatrix = TrajectoryPlan.minAccelnPath(Drone,des_state,plan_t,t)

if PLAN_TRAJ[0] == True and PLAN_TRAJ[1] == 3:
    des_stateMatrix = TrajectoryPlan.minJerkPath(Drone,des_state,plan_t,t)

#des_stateMatrix = TrajectoryPlan.minVelPath(Drone,des_state,plan_t,t)

#des_stateMatrix = TrajectoryPlan.minAccelnPath(Drone,des_state,plan_t,t)

#des_stateMatrix = TrajectoryPlan.minJerkPath(Drone,des_state,plan_t,t)
print("Trajectory Planning Done. Solving for the states")
#np.savetxt('check.csv',des_stateMatrix,fmt='%f',delimiter=",")
solvedState = simulate(TUNING_MATRIX,des_stateMatrix,t)
print("Simulation Done. Animating the Plot")

anime()
plotResults(solvedState,des_stateMatrix,t)