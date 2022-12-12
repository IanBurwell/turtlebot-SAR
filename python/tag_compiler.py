
import aglomerate_apriltag_pose
import heapq as hq

def tag_check_new(tag_ID,tag_dict):

    if tag_ID in tag_dict:
        NEW = False
    else:
        NEW = True

    return NEW

def tag_detected(tag_ID,T_c_tag,T_w_tag,max_detect_dis, max_detect_angle, max_comp_dis, max_comp_angle,tag_dict,tag_queue):
    #Checks if detection is valid (not necessarily valid for composition)
    if aglomerate_apriltag_pose.check_valid_measurement(T_c_tag,max_detect_angle,max_detect_angle,max_detect_dis):
        #Checks if tag is already in tag dictionary(Already detected) and add it and world frame pose to list if it isn't
        #Also will add it to queue
        if tag_check_new(tag_ID,tag_dict):
            tag_dict.update({tag_ID:(T_w_tag,0)})
            hq.heappush(tag_queue, tag_ID)

        #Checks if measurement is valid for updating compiled pose estimate
        if aglomerate_apriltag_pose.check_valid_measurement(T_c_tag,max_comp_angle,max_comp_angle, max_comp_dis):
                comp_tag = aglomerate_apriltag_pose.average_tag(T_w_tag,tag_dict[tag_ID])
                tag_dict.update({tag_ID:comp_tag})

        return True
    else:
        return False



def find_goal_pose(T_rob_tag):
