#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from jupiterobot2_msgs.msg import Mediapipe_Pose
from cv_bridge import CvBridge

class PerceptionAgent:
    def __init__(self):
        rospy.init_node('perception_agent', anonymous=True)
        self.bridge = CvBridge()
        # 原始输入
        rospy.Subscriber('/camera/color/image_raw', Image,   self.image_cb,  queue_size=1)
        rospy.Subscriber('/mediapipe/pose',        Mediapipe_Pose, self.pose_cb,   queue_size=1)
        # 转发到 Agent 总线上
        self.pub_img  = rospy.Publisher('/perception/image', Image,           queue_size=1)
        self.pub_pose = rospy.Publisher('/perception/pose', Mediapipe_Pose,  queue_size=1)
        rospy.loginfo("PerceptionAgent started.")
        rospy.spin()

    def image_cb(self, msg: Image):
        self.pub_img.publish(msg)

    def pose_cb(self, msg: Mediapipe_Pose):
        self.pub_pose.publish(msg)

if __name__ == '__main__':
    try:
        PerceptionAgent()
    except rospy.ROSInterruptException:
        pass
