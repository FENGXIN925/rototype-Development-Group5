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
import math
import cv2
import requests
import os

class FallDetectionNode:
    # —— TUNABLE PARAMETERS ——
    MAX_HEAD_FEET_DIST = 600     # px
    FAST_FALL_VELOCITY = 150     # px/sec
    HORIZONTAL_ANGLE = 60        # deg: >60° → torso nearly horizontal
    BENDING_ANGLE = 30           # deg: between 30°–60° → bent over
    HIP_GROUND_DIST = 100        # px
    STATIC_VARIATION = 20        # px variation considered “static”
    STATIC_TIME = 1.0            # sec before confirming a level
    COOLDOWN = 10                # sec between alerts of same level

    def __init__(self):
        rospy.init_node('fall_detection_node_precise', anonymous=True)
        # State
        self.state = "NORMAL"
        self.t0 = time.time()
        self.cooldowns = {lvl: 0 for lvl in ("SUDDEN_LIE", "LYING_STATIC", "SITTING", "BENDING")}
        # Pose & image
        self.pose_history = deque(maxlen=int(self.STATIC_TIME * 10))
        self.nose_hist = deque(maxlen=20)
        self.bridge = CvBridge()
        self.latest_frame = None

        # Subscribers & Publishers
        rospy.Subscriber("/mediapipe/pose", Mediapipe_Pose, self.pose_cb)
        rospy.Subscriber("/mediapipe/image_raw", Image, self.img_cb)
        self.alert_pub = rospy.Publisher("/fall_alert", String, queue_size=10)
        self.sound = SoundClient()

        # Telegram
        self.bot_token = "8170738660:AAECRZcR__1I3R1HWZXOn7G3uKQAOsLyyFY"
        self.chat_id   = "6785492558"

        rospy.loginfo("📢 [FallDetectionNode] Starting with refined levels…")
        self.loop_rate = rospy.Rate(10)
        self.main_loop()

    def pose_cb(self, msg):
        now = time.time()
        # key landmarks
        nx, ny = msg.nose.x, msg.nose.y
        lx, ly = msg.left_hip.x, msg.left_hip.y
        rx, ry = msg.right_hip.x, msg.right_hip.y
        lsx, lsy = msg.left_shoulder.x, msg.left_shoulder.y
        rsx, rsy = msg.right_shoulder.x, msg.right_shoulder.y
        ax, ay = max(msg.left_ankle.x, msg.right_ankle.x), max(msg.left_ankle.y, msg.right_ankle.y)

        # mean points
        hip_x = (lx + rx) / 2.0
        hip_y = (ly + ry) / 2.0
        shoulder_x = (lsx + rsx) / 2.0
        shoulder_y = (lsy + rsy) / 2.0

        # head-feet distance
        head_dist = abs(ny - ay)

        # torso angle relative to vertical (0° = upright; 90° = horizontal)
        dx = shoulder_x - hip_x
        dy = shoulder_y - hip_y
        angle = abs(math.degrees(math.atan2(abs(dx), abs(dy))))

        # nose velocity
        self.nose_hist.append((ny, now))
        while len(self.nose_hist)>=2 and now - self.nose_hist[0][1] > 0.5:
            self.nose_hist.popleft()
        if len(self.nose_hist)>=2:
            y0, t0 = self.nose_hist[0]
            y1, t1 = self.nose_hist[-1]
            vel = (y1 - y0) / (t1 - t0) if t1>t0 else 0
        else:
            vel = 0

        # save for history-based static check
        self.pose_history.append((head_dist, hip_y-ay, angle))

        # store for classification
        self.current = {
            "head_dist": head_dist,
            "hip_ground": abs(hip_y - ay),
            "angle": angle,
            "vel": vel,
            "time": now
        }

    def img_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Img cb error: {e}")

    def is_static(self):
        dists = [d for d,_,_ in self.pose_history]
        return (max(dists) - min(dists)) < self.STATIC_VARIATION

    def send_telegram(self, text):
        try:
            if self.latest_frame is not None:
                tmp = "/tmp/fall.jpg"
                cv2.imwrite(tmp, self.latest_frame)
                files = {'photo': open(tmp,'rb')}
                data = {'chat_id':self.chat_id, 'caption':text}
                requests.post(f"https://api.telegram.org/bot{self.bot_token}/sendPhoto",
                              files=files, data=data)
                os.remove(tmp)
            else:
                data = {'chat_id':self.chat_id, 'text':text}
                requests.post(f"https://api.telegram.org/bot{self.bot_token}/sendMessage", data=data)
        except Exception as e:
            rospy.logerr(f"Telegram error: {e}")

    def classify(self):
        c = self.current
        now = c["time"]
        # 1️⃣ Sudden Lie: fast drop + near-horizontal torso
        if c["head_dist"] < self.MAX_HEAD_FEET_DIST and c["vel"] > self.FAST_FALL_VELOCITY and c["angle"] > self.HORIZONTAL_ANGLE:
            return "SUDDEN_LIE"
        # 2️⃣ Static Lying: torso near-horizontal & static for a bit
        if c["angle"] > self.HORIZONTAL_ANGLE and self.is_static():
            return "LYING_STATIC"
        # 3️⃣ Sitting: hip near ground + torso between horizontal and upright
        if c["hip_ground"] < self.HIP_GROUND_DIST and self.BENDING_ANGLE < c["angle"] <= self.HORIZONTAL_ANGLE:
            return "SITTING"
        # 4️⃣ Bending: torso bent but not seated
        if self.BENDING_ANGLE < c["angle"] <= self.HORIZONTAL_ANGLE:
            return "BENDING"
        # ✔️ Normal
        return "NORMAL"

    def main_loop(self):
        while not rospy.is_shutdown():
            if not hasattr(self, 'current'):
                self.loop_rate.sleep()
                continue

            new_state = self.classify()
            t = self.current["time"]
            # if state changed, reset timer
            if new_state != self.state:
                self.state = new_state
                self.t0 = t
                rospy.loginfo(f"➡️ State → {self.state}")

            elapsed = t - self.t0
            # handle alerts with cooldown
            for lvl, text, voice, delay in [
                ("SUDDEN_LIE",   "🚨 Sudden Fall Detected!",    "Sudden fall detected. Help needed immediately.", 0.5),
                ("LYING_STATIC","🛌 Person Lying Static.",    "Person is lying down for too long.",         1.0),
                ("SITTING",     "🪑 Person Sitting on Floor.", "Detected a sitting fall.",                    1.0),
                ("BENDING",     "🤸 Abnormal Bending Posture.", "Person is bent over unusually.",              2.0),
            ]:
                if self.state==lvl and elapsed>delay and (t - self.cooldowns[lvl])>self.COOLDOWN:
                    self.alert_pub.publish(text)
                    self.sound.say(voice)
                    self.send_telegram(text)
                    self.cooldowns[lvl] = t

            self.loop_rate.sleep()

if __name__=='__main__':
    try:
        FallDetectionNode()
    except rospy.ROSInterruptException:
        pass
