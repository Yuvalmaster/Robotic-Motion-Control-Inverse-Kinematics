"""
Created on Fri Apr 29 10:51:26 2022

@author: Yuval Argoetti

"""
# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
'PACKAGES'
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d                # Package for 3D plotting

# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
class robot_arm():
    'Define Robot Constants'
    l1 = 1; l2 = 0.8; l3 = 0.7;
    lim_min = np.array([-np.pi, 0, -np.pi])     # Joints min angles
    lim_max = np.array([np.pi, np.pi, np.pi])   # Joints max angles

    'Define Obstacle locations'
    obs_radius = 0.2                            # Radius of both obstacles
    obs1 = [0.05,1]                             # x,y coordinations of obstacle 1
    obs2 = [-0.6,0.6]                           # x,y coordinations of obstacle 2
    obs3 = [-0.3,0.8]                           # x,y coordinations of obstacle 3 Q3
    
    'Define parameters for attraction-repulsion Movement'
    AP = 5.0                                    # Attraction potential factor 
    RP = 0.08                                   # Repulsion potential factor   
    Q = 0.1                                     # Repulsion Zone boundries
    alpha = 0.001                               # Step size
# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #  
    def __init__(self):
        print('Man is a robot with defects - I, Robot')    

# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #    
    # Generate a random robot configuration (joint angles) within the limits
    def gen_rand_conf(self):
        return np.random.random((3,)) * (self.lim_max - self.lim_min).reshape((3,)) + self.lim_min.reshape((3,))
    
# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    # Direct kinematics function - receives the angles of the robot and returns the position of the end-effector
    def direct_kinematics(self, angle):
        x = np.cos(angle[0]) * self.l1 + np.cos(angle[0]+angle[1])* self.l2 +  np.cos(angle[0] +angle[1] + angle[2])* self.l3 
        y = np.sin(angle[0]) * self.l1 + np.sin(angle[0]+angle[1])* self.l2 +  np.sin(angle[0] +angle[1] + angle[2])* self.l3 
        theta = np.sum(angle[0] + angle[1] + angle[2]) 
        return np.array([x, y, theta])          # Position of end-effector

# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #    
    # Checks if the end-effector is in collision with the obstacle, returns: True - in collision, False - no collision
    def collision_checker_Q1_Q2(self, angle):
        
        Rob_coor = self.direct_kinematics(angle) # Load the robot coordinates from direct kinematics
        
        # Check if collision with obstacle
        if (np.linalg.norm(Rob_coor[0:1] - self.obs1) < self.obs_radius      # Calculate if the end-effector collide with obs1
            or np.linalg.norm(Rob_coor[0:1] - self.obs2) < self.obs_radius   # Calculate if the end-effector collide with obs2
            or Rob_coor[1] <= 0.4                                            # Calculate if the end-effector cross lower y-axis limit
            or Rob_coor[1] >= 1.2 ):                                         # Calculate if the end-effector cross upper y-axis limit
            return True                                                      # Collide or cross limits
        
        # Check if angles in limit
        if np.any(angle < self.lim_min) or np.any(angle > self.lim_max):
            return True                                                      # Collide or cross limits
        
        return False                                                         # No collsion
    
# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #    
    # Checks if the end-effector is in collision with the obstacle, returns: True - in collision, False - no collision
    def collision_checker_Q3(self, angle):
        
        Rob_coor = self.direct_kinematics(angle)                             # Load the robot coordinates from direct kinematics
        
        # Check if collision with obstacle
        if (np.linalg.norm(Rob_coor[:2] - self.obs3) < self.obs_radius       # Calculate if the end-effector collide with obs3
            or Rob_coor[1] <= 0.4                                            # Calculate if the end-effector cross lower y-axis limit
            or Rob_coor[1] >= 1.2 ):                                         # Calculate if the end-effector cross upper y-axis limit
            return True                                                      # Collide or cross limits
        
        # Check if angles in limit
        if np.any(angle < self.lim_min) or np.any(angle > self.lim_max):
            return True                                                      # Collide or cross limits
        
        return False        

# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #    
    def Attraction_Potencial(self, end_effector_coordinates, target): 
        return  self.AP * (end_effector_coordinates - target)                 
 
    
# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #     
    def Repulsive_Potential(self, end_effector_coordinates): 
        F = np.zeros((2,)) 
        
        # If the end-effector is within the repulsion zone of obs3
        d_obs3 = np.linalg.norm(end_effector_coordinates - self.obs3) 
        if d_obs3 < self.obs_radius + self.Q: 
            F += 2 * self.RP  * (1/self.Q - 1/d_obs3) * (1/d_obs3**2) * end_effector_coordinates 
        
        # If the end-effector is within the repulsion zone of upper limit
        d_upper = np.abs(end_effector_coordinates[1] - 1.2) 
        if d_upper < self.Q: 
            F += [0, 2 * self.RP * (1/self.Q - 1/d_upper) * (1/d_upper**2) * end_effector_coordinates[1]] 
        
        # If the end-effector is within the repulsion zone of lower limit
        d_lower = np.abs(end_effector_coordinates[1] - 0.4) 
        if d_lower < self. Q: 
            F += [0, 2 * self.RP * (1/self.Q - 1/d_lower) * (1/d_lower**2) * end_effector_coordinates[1]] 
        
        return F
 


# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #     
'Preperation'
R = robot_arm()                                     # Generate a robot
A_start = [0.5,1,np.arctan2(0.5,1)]                 # Starting coordinates
A_end = [-1.3,0.5,np.arctan2(-1.3,0.5)-np.pi]       # End coordinates


# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #     
'Plot Qfree' 
# Create an array of allowed angles in space for the robot
Q_free = [] 
while len(Q_free) < 20000:
    angles = R.gen_rand_conf()
    if R.collision_checker_Q1_Q2(angles) == False:
        Q_free.append(angles)

Q_free = np.array(Q_free) 

# Plot Data
plt.figure() ; axs = plt.axes(projection='3d') 
axs.plot3D(Q_free[:,0], Q_free[:,1], Q_free[:,2], '.', markersize = 0.2) 
axs.set_xlabel('Angle 1 [Rad]') ; axs.set_ylabel('Angle 2 [Rad]') 
axs.set_zlabel('Angle 3 [Rad]') ; axs.set_title('Allowed motors angles synchronization space')
plt.show() 

# --------------------------------------------------------------------------------------------------------------------------------------------------------------------- #     
'Robot Movement'

# Plot space and bounderies
plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
fig, axs = plt.subplots() 

plt.plot([-1.5, 1.5], [1.2, 1.2], '-g')                         # Upper limit
plt.plot([-1.5, 1.5], [0.4, 0.4], '-g')                         # Lower limit
plt.plot(R.obs3[0], R.obs3[1],'.k')

plt.plot(A_start[0], A_start[1], '.b', markersize = 15)         # Start coordinations
plt.plot(A_end[0], A_end[1], '.r', markersize = 15)             # End coordinations
for i, j in zip([A_start[0],A_end[0],R.obs3[0]], 
                [A_start[1],A_end[1],R.obs3[1]]):
   
    plt.text(i-0.17, j+0.05, '({}, {})'.format(i, j))

Obstacle = plt.Circle((R.obs3[0], R.obs3[1]),
                       R.obs_radius, color='g') 
axs.add_artist(Obstacle)                                        # Add Circle

# Plan Movement
end_effector_coordinates = np.copy(A_start[0:2]) 
end_effector_coor_vec = [] 

while 1: 
    end_effector_coor_vec.append(np.copy(end_effector_coordinates)) 
    F = R.Attraction_Potencial(end_effector_coordinates, A_end[0:2]) + R.Repulsive_Potential(end_effector_coordinates) 
    end_effector_coordinates -= R.alpha * F 
    
    if np.linalg.norm(F) < 0.03: 
        end_effector_coor_vec.append(np.copy(end_effector_coordinates)) 
        break 

end_effector_coor_vec = np.array(end_effector_coor_vec) 
plt.plot(end_effector_coor_vec[:,0], end_effector_coor_vec[:,1], '.-') 
plt.axis('equal')  ; plt.xlabel('X Coordinate') ; plt.ylabel('Y Coordinate')
plt.title('End-effector movemet planning') ; plt.show()
 