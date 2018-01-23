#!/usr/bin/env python

import sys
import rospy
from thormang_ik_api.srv import *

whichArm = 'left'
currJointPose = [ ('l_arm_wr_p', 0.0851721),\
                  ('l_arm_wr_y', -0.157429),\
                  ('l_arm_wr_r', 1.04109),  \
                  ('l_arm_el_y', 0.494452), \
                  ('l_arm_sh_r', 1.03166),  \
                  ('l_arm_sh_p1', 0.568977),\
                  ('l_arm_sh_p2', -0.115366) ]

targetPosition = [ 0.300, 0.310, 0.799 ]
targetOrientation = [ -0.00014836, 0.000113437, 6.11034e-05, 1.0 ]
desiredPose = targetPosition + targetOrientation

def create_msg(whichArm, currJointPose, desiredPose):
    wa = std_msgs.msg.String(whichArm)
    cjp = []
    time = rospy.get_time()
    for jp in currJointPose:
        cjp.append(thormang3_manipulation_module_msgs.msg.JointPose(jp[0],jp[1],time))
    tp = geometry_msgs.msg.Point(
            x=targetPosition[0],y=targetPosition[1],z=targetPosition[2])
    to = geometry_msgs.msg.Quaternion(\
            x=targetOrientation[0],y=targetOrientation[1],\
            z=targetOrientation[2],w=targetOrientation[3])
    dp = geometry_msgs.msg.Pose(tp, to)
    #dp = thormang3_manipulation_module_msgs.msg.KinematicsPose(pose=desiredPose)
    return wa, cjp, dp

def call_ik_service():
    rospy.init_node('demo', anonymous=True)
    print('Waiting for the thormang_ik_api/calc_ik service')
    rospy.wait_for_service('thormang_ik_api/calc_ik')
    print('Creating the message')
    wa, cjp, dp = create_msg(whichArm, currJointPose, targetPosition)
    print('Calling the thormang_ik_api/calc_ik service')
    calc_ik_function = rospy.ServiceProxy('thormang_ik_api/calc_ik', calc_ik)
    res = calc_ik_function(wa,cjp,dp)
    print(res)

if __name__ == '__main__':
    call_ik_service()

