#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from jupiterobot2_msgs.msg import Mediapipe_Pose
from sound_play.libsoundplay import SoundClient
from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import requests 
import os

class FallDetectionNode:
    def __init__(self):
        self.ready = False
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

        self.previous_nose_y = None
        self.nose_velocity = 0
        self.nose_history = deque(maxlen=20)  # 存过去的鼻子y坐标 + 时间戳

        self.telegram_token = "8170738660:AAECRZcR__1I3R1HWZXOn7G3uKQAOsLyyFY"
        self.chat_id = "6785492558"
        self.camera_id = 0  # /dev/video0

        self.cooldowns = {
            "LEVEL_1_FALL": 0,
            "LEVEL_2_SITTING_FALL": 0,
            "LEVEL_3_LOW_POSTURE": 0
        }
        self.cooldown_interval = 10  # seconds

        rospy.loginfo(" Fall Detection Node with Telegram + English Voice Alerts Started")
        # 摄像头图像桥接器 + 订阅
        self.bridge = CvBridge()
        self.latest_frame = None
        rospy.Subscriber("/mediapipe/image_raw", Image, self.image_callback)

        self.rate = rospy.Rate(10)
        self.ready = True
        self.main_loop()
        

    def pose_callback(self, msg):
        if not self.ready:
            return
        self.nose_y = msg.nose.y
        self.ankle_y = max(msg.left_ankle.y, msg.right_ankle.y)
        self.hip_y = (msg.left_hip.y + msg.right_hip.y) / 2.0

        # 更新历史记录（用于计算过去 0.5 秒速度）
        now = time.time()
        self.nose_history.append((self.nose_y, now))

        # 保留最近 0.5 秒内的点
        while len(self.nose_history) >= 2 and now - self.nose_history[0][1] > 0.5:
            self.nose_history.popleft()

        # 计算速度（单位：像素/秒）
        if len(self.nose_history) >= 2:
            y_old, t_old = self.nose_history[0]
            y_new, t_new = self.nose_history[-1]
            if t_new - t_old > 0:
                self.nose_velocity = (y_new - y_old) / (t_new - t_old)
            else:
                self.nose_velocity = 0
        else:
            self.nose_velocity = 0


    def is_pose_static(self):
        ys = [nose for nose, _, _ in self.pose_history]
        return max(ys) - min(ys) < 20

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Image callback error: {e}")


    def send_telegram_alert(self, message):
        try:
            rospy.loginfo(" Sending Telegram alert...")
            image_sent = False

            if self.latest_frame is not None:
                img_path = "/tmp/fall_alert.jpg"
                cv2.imwrite(img_path, self.latest_frame)
                files = {'photo': open(img_path, 'rb')}
                data = {'chat_id': self.chat_id, 'caption': message}
                response = requests.post(f"https://api.telegram.org/bot{self.telegram_token}/sendPhoto", files=files, data=data)
                os.remove(img_path)
                rospy.loginfo(f" Image sent response: {response.status_code}, {response.text}")
                image_sent = True
            else:
                rospy.logwarn(" latest_frame is None, skipping image.")

            if not image_sent:
                data = {'chat_id': self.chat_id, 'text': message}
                response = requests.post(f"https://api.telegram.org/bot{self.telegram_token}/sendMessage", data=data)
                rospy.loginfo(f" Fallback text sent: {response.status_code}, {response.text}")

        except Exception as e:
            rospy.logerr(f" Telegram alert failed: {e}")


    def classify_fall(self):
        if None in (self.nose_y, self.hip_y, self.ankle_y):
            rospy.logwarn(" Coordinates not ready, skipping detection.")
            return "NORMAL"

        height = abs(self.nose_y - self.ankle_y)
        head_near_feet = abs(self.nose_y - self.ankle_y) < 150
        hip_near_ground = abs(self.hip_y - self.ankle_y) < 100
        nose_low = abs(self.nose_y - self.hip_y) < 600
        fast_fall = self.nose_velocity > 100
        

        rospy.loginfo(f" Points: nose_y={self.nose_y:.1f}, hip_y={self.hip_y:.1f}, ankle_y={self.ankle_y:.1f}")
        rospy.loginfo(f" Conditions: head_near_feet={head_near_feet}, hip_near_ground={hip_near_ground}, fast_fall={fast_fall}")

        if height < 600 and fast_fall:
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
                rospy.loginfo(f" State changed to: {self.state}")

            now = time.time()
            time_in_state = now - self.state_enter_time if self.state_enter_time else 0

            if self.state == "LEVEL_1_FALL" and time_in_state >= 1:
                if now - self.cooldowns["LEVEL_1_FALL"] > self.cooldown_interval:
                    msg = " Level 1 Fall Detected: Lying Down."
                    self.alert_pub.publish(msg)
                    self.soundhandle.say("Level one fall detected. Please check immediately.")
                    self.send_telegram_alert(msg)
                    self.cooldowns["LEVEL_1_FALL"] = now

            elif self.state == "LEVEL_2_SITTING_FALL" and time_in_state >= 1:
                if now - self.cooldowns["LEVEL_2_SITTING_FALL"] > self.cooldown_interval:
                    msg = " Level 2 Fall: Sitting on the ground."
                    self.alert_pub.publish(msg)
                    self.soundhandle.say("Level two fall. The person is sitting on the ground.")
                    self.send_telegram_alert(msg)
                    self.cooldowns["LEVEL_2_SITTING_FALL"] = now

            elif self.state == "LEVEL_3_LOW_POSTURE" and time_in_state >= 3 and self.is_pose_static():
                if now - self.cooldowns["LEVEL_3_LOW_POSTURE"] > self.cooldown_interval:
                    msg = " Level 3: Abnormal posture held too long."
                    self.alert_pub.publish(msg)
                    self.soundhandle.say("Low posture detected for a long time.")
                    self.send_telegram_alert(msg)
                    self.cooldowns["LEVEL_3_LOW_POSTURE"] = now

            self.rate.sleep()

if __name__ == '__main__':
    try:
        FallDetectionNode()
    except rospy.ROSInterruptException:
        pass
