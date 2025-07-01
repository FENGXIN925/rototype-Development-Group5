#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from jupiterobot2_msgs.msg import Mediapipe_Pose
from sound_play.libsoundplay import SoundClient
from collections import deque
import time

class FallDetectionNode:
    def __init__(self):
        rospy.init_node('fall_detection_node', anonymous=True)

        rospy.Subscriber("/mediapipe/pose", Mediapipe_Pose, self.pose_callback)

        self.alert_pub = rospy.Publisher("/fall_alert", String, queue_size=10)
        self.soundhandle = SoundClient()
        rospy.sleep(1)

        self.state = "NORMAL"
        self.state_enter_time = None
        self.pose_history = deque(maxlen=30)

        self.nose_y = None
        self.ankle_y = None
        self.hip_y = None

        rospy.loginfo("✅ Fall Detection Node with English Voice Alerts Started")
        self.rate = rospy.Rate(10)
        self.main_loop()

    def pose_callback(self, msg):
        # 解析头、髋、脚踝关键点
        self.nose_y = msg.nose.y
        self.ankle_y = max(msg.left_ankle.y, msg.right_ankle.y)
        self.hip_y = (msg.left_hip.y + msg.right_hip.y) / 2.0

    def is_pose_static(self):
        ys = [nose for nose, _, _ in self.pose_history]
        return max(ys) - min(ys) < 20

    def classify_fall(self):
        if None in (self.nose_y, self.hip_y, self.ankle_y):
            rospy.logwarn("⛔ Coordinates not ready, skipping detection.")
            return "NORMAL"

        height = abs(self.nose_y - self.ankle_y)
        head_below_feet = self.nose_y > self.ankle_y
        hip_near_ground = abs(self.hip_y - self.ankle_y) < 100
        nose_low = abs(self.nose_y - self.hip_y) < 600

        rospy.loginfo(f"📌 Points: nose_y={self.nose_y:.1f}, hip_y={self.hip_y:.1f}, ankle_y={self.ankle_y:.1f}")
        rospy.loginfo(f"➡️ Conditions: head below feet={head_below_feet}, height={height:.1f}, hip near ground={hip_near_ground}, nose near hip={nose_low}")

        if head_below_feet and height < 600:
            return "LEVEL_1_FALL"
        elif hip_near_ground and nose_low:
            return "LEVEL_2_SITTING_FALL"
        elif hip_near_ground:
            return "LEVEL_3_LOW_POSTURE"
        else:
            return "NORMAL"

    def main_loop(self):
        while not rospy.is_shutdown():
            if None in (self.nose_y, self.hip_y, self.ankle_y):
                self.rate.sleep()
                continue

            self.pose_history.append((self.nose_y, self.hip_y, self.ankle_y))
            new_state = self.classify_fall()

            if new_state != self.state:
                self.state = new_state
                self.state_enter_time = time.time()
                rospy.loginfo(f"🔄 State changed to: {self.state}")

            time_in_state = time.time() - self.state_enter_time if self.state_enter_time else 0

            if self.state == "LEVEL_1_FALL":
                if time_in_state >= 1:
                    self.alert_pub.publish("‼️ Fall - Lying Down (Level 1)")
                    self.soundhandle.say("Level one fall detected. Please check immediately.")
                    rospy.logwarn("‼️ Level 1 Fall Alert")

            elif self.state == "LEVEL_2_SITTING_FALL":
                if time_in_state >= 1:
                    self.alert_pub.publish("⚠️ Fall - Sitting on the ground (Level 2)")
                    self.soundhandle.say("Level two fall. The person is sitting on the ground.")
                    rospy.logwarn("⚠️ Level 2 Sitting Fall Alert")

            elif self.state == "LEVEL_3_LOW_POSTURE":
                if time_in_state >= 1 and self.is_pose_static():
                    self.alert_pub.publish("📌 Abnormal posture held too long (Level 3)")
                    self.soundhandle.say("Low posture detected for a long time.")
                    rospy.loginfo("📌 Level 3 Low Posture Alert")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        FallDetectionNode()
    except rospy.ROSInterruptException:
        pass
