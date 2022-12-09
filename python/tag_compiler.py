
import aglomerate_apriltag_pose

def tag_check_new(tag_ID,tag_list):

    if tag_ID in tag_list:
        NEW = False
    else:
        NEW = True
        tag_list.append(tag_ID)

    return(NEW, tag_list)

