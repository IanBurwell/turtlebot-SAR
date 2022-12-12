#!/usr/env/python3

import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult
from actionlib_msgs.msg import GoalID, GoalStatusArray

class MovementIntermediary(): 
    def __init__(self):
        self.goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
        self.feedback_pub = rospy.Publisher('movement_intermediary/feedback', MoveBaseActionFeedback, queue_size=10)
        self.status_pub = rospy.Publisher('movement_intermediary/status', GoalStatusArray, queue_size=10)
        self.result_pub = rospy.Publisher('movement_intermediary/result', MoveBaseActionResult, queue_size=10)

        self.goal = rospy.Subscriber('movement_intermediary/goal', MoveBaseActionGoal, self.send_goal_to_move_base)
        self.cancel = rospy.Subscriber('movement_intermediary/cancel', GoalID, self.send_cancel_to_move_base)
        self.feedback = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.send_feedback_to_explore_lite)
        self.status = rospy.Subscriber('move_base/status', GoalStatusArray, self.send_status_to_explore_lite)
        self.result = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.send_result_to_explore_lite)  
        
        rospy.init_node('movement_intermediary')
        rospy.spin()

    def send_goal_to_move_base(self, msg):
        self.goal_pub.publish(msg)
    
    def send_cancel_to_move_base(self, msg):
        self.cancel_pub.publish(msg)
    
    def send_feedback_to_explore_lite(self, msg):
        self.feedback_pub.publish(msg)

    def send_status_to_explore_lite(self, msg):
        self.status_pub.publish(msg)

    def send_result_to_explore_lite(self, msg):
        self.result_pub.publish(msg)

try:
    MovementIntermediary()
except rospy.ROSInterruptException:
    pass
