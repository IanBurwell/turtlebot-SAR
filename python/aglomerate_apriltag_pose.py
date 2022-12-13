
import numpy as np
from geometry_msgs.msg import Pose

#Determines how far april tag measurement is from z-axis of camera (~x-axis of robot frame) + Distance
def calc_z_deviation(temp_tag_pos):
    #horizontal angle
    theta = np.arctan2(temp_tag_pos.x, temp_tag_pos.z)
    #vertical angle
    phi = np.arctan2(temp_tag_pos.y,temp_tag_pos.z)
    #Distance
    dis = np.sqrt(temp_tag_pos.x**2 + temp_tag_pos.y**2 + temp_tag_pos.z**2)

    return [theta, phi, dis]

#Calculate postions for turtlebot to go to for good tag measurements in tag frame
def calc_valid_R_pos(max_dis,min_dis,max_valid_angle,T_c_tag,N):
    #Valid Position in tag frame
    theta = np.random.uniform(-max_valid_angle,max_valid_angle,N)
    dis = np.random.uniform(min_dis,max_dis,N)
    z= dis*np.cos(theta)
    x = dis*np.sin(theta)
    y = -T_c_tag.position.y
    pos_tag_target = [x,y,z]

    return pos_tag_target

#Given target position and tag Pose in turtlebot frame, calculate optimal orientation quaternian for tag measurements
def calc_optimal_R_orientation(T_rob_tag,target_pos_r):

    tag_pos = T_rob_tag.Point
    t = tag_pos-target_pos_r
    theta = np.arctan2(t[1],t[0])
    quat = [0,0,np.sin(theta/2),np.cos(theta/2)]

    return quat

def average_tag(T_w_tag_new, tag_entry):
   
    T_w_tag_new = T_w_tag_new.pose

    (T_w_tag, data_num) = tag_entry

    T_av = Pose()

    if data_num == 0:
        T_av = T_w_tag_new
    else:
        T_av.position.x = (T_w_tag_new.position.x + data_num * T_w_tag.position.x)/(data_num+1)
        T_av.position.y = (T_w_tag_new.position.y + data_num * T_w_tag.position.y)/(data_num+1)
        T_av.position.z = (T_w_tag_new.position.z + data_num * T_w_tag.position.z)/(data_num+1)

        T_av.orientation.x = (T_w_tag_new.orientation.x + data_num * T_w_tag.orientation.x)/(data_num+1)
        T_av.orientation.y = (T_w_tag_new.orientation.y + data_num * T_w_tag.orientation.y)/(data_num+1)
        T_av.orientation.z = (T_w_tag_new.orientation.z + data_num * T_w_tag.orientation.z)/(data_num+1)
        T_av.orientation.w = (T_w_tag_new.orientation.w + data_num * T_w_tag.orientation.w)/(data_num+1)

        # normalize quaternion
        norm = np.sqrt(T_av.orientation.x**2 + T_av.orientation.y**2 + T_av.orientation.z**2 + T_av.orientation.w**2)
        T_av.orientation.x = T_av.orientation.x/norm
        T_av.orientation.y = T_av.orientation.y/norm
        T_av.orientation.z = T_av.orientation.z/norm
        T_av.orientation.w = T_av.orientation.w/norm

    return (T_av, data_num+1)

def check_valid_measurement(T_c_tag,max_theta,max_phi,max_dis):

    [theta,phi,dis] = calc_z_deviation(T_c_tag.position)

    if (abs(theta) <= max_theta) & (abs(phi) <= max_phi) & (dis <= max_dis):
        return True
    else:
        return False

#check if target position is occupied
#def check_valid_pos():