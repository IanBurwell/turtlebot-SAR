
import numpy as np

#Determines how far april tag measurement is from z-axis of camera (~x-axis of robot frame) + Distance
def calc_z_deviation(temp_tag_pos):
    #horizontal angle
    theta = np.arctan2(temp_tag_pos[0],temp_tag_pos[2])
    #vertical angle
    phi = np.arctan2(temp_tag_pos[1],temp_tag_pos[2])
    #Distance
    dis = np.sum(temp_tag_pos**2)

    return [theta, phi, dis]

#Calculate postions for turtlebot to go to for good tag measurements in tag frame
def calc_valid_R_pos(max_dis,min_dis,max_valid_angle,T_c_tag,N):
    #Valid Position in tag frame
    theta = np.random.uniform(-max_valid_angle,max_valid_angle,N)
    dis = np.random.uniform(min_dis,max_dis,N)
    z= dis*np.cos(theta)
    x = dis*np.sin(theta)
    y = -T_c_tag.Point[2]
    pos_tag_target = [x,y,z]

    return pos_tag_target

#Given target position and tag Pose in turtlebot frame, calculate optimal orientation quaternian for tag measurements
def calc_optimal_R_orientation(T_rob_tag,target_pos_r):

    tag_pos = T_rob_tag.Point
    t = tag_pos-target_pos_r
    theta = np.arctan2(t[1],t[0])
    quat = [0,0,np.sin(theta/2),np.cos(theta/2)]

    return quat

def average_tag(T_w_tag_new,T_w_tag,data_num):

    T_av = (T_w_tag_new + data_num*T_w_tag)/(data_num+1)

    return (T_av, data_num+1)

def check_valid_meaurement(T_c_tag,max_theta,max_phi,max_dis):

    [theta,phi,dis] = calc_z_deviation(T_c_tag)

    if (abs(theta) <= max_theta) & (abs(phi) <= max_phi) & (dis <= max_dis):
        return True
    else:
        return False

#check if target position is occupied
def check_valid_pos():