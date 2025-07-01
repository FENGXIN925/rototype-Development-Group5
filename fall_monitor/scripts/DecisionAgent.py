#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, time
from sensor_msgs.msg import Image
from jupiterobot2_msgs.msg import Mediapipe_Pose, FallEvent
from ultralytics import YOLO
from cv_bridge import CvBridge
from collections import deque

class DecisionAgent:
    def __init__(self):
        rospy.init_node('decision_agent', anonymous=True)
        # 从参数读取你的 YOLO 模型路径
        model_path = rospy.get_param('~yolo_model_path', '/path/to/yolov8n_fall.pt')
        self.yolo = YOLO(model_path)
        self.bridge = CvBridge()

        # 缓存最新数据
        self.latest_image = None
        self.latest_pose  = None

        # 计算鼻子速度用
        self.nose_hist = deque(maxlen=20)
        self.nose_vel = 0.0

        # 订阅合成数据
        rospy.Subscriber('/perception/image', Image,   self.image_cb,  queue_size=1)
        rospy.Subscriber('/perception/pose',  Mediapipe_Pose, self.pose_cb,  queue_size=1)

        # 发布最终跌倒事件
        self.pub_evt = rospy.Publisher('/fall_event', FallEvent, queue_size=1)

        self.rate = rospy.Rate(10)
        rospy.loginfo("DecisionAgent started.")
        self.main_loop()

    def image_cb(self, msg: Image):
        self.latest_image = msg

    def pose_cb(self, msg: Mediapipe_Pose):
        self.latest_pose = msg
        # 更新鼻子历史，用于速度计算
        now = time.time()
        self.nose_hist.append((msg.nose.y, now))
        # 保留 0.5s 内
        while len(self.nose_hist) >= 2 and now - self.nose_hist[0][1] > 0.5:
            self.nose_hist.popleft()
        if len(self.nose_hist) >= 2:
            y0, t0 = self.nose_hist[0]
            y1, t1 = self.nose_hist[-1]
            self.nose_vel = (y1 - y0) / (t1 - t0) if t1 > t0 else 0.0

    def classify_level(self, yolo_flag: bool) -> int:
        """
        返回 0~5 的跌倒等级：
          0 – 正常
          1 – 轻微倾斜
          2 – 半蹲/模糊坐姿
          3 – 坐地
          4 – 平卧
          5 – 快速坠落（严重平卧）
        """
        if not self.latest_pose:
            return 0
        nose = self.latest_pose.nose.y
        hip  = (self.latest_pose.left_hip.y + self.latest_pose.right_hip.y) / 2.0
        ank  = max(self.latest_pose.left_ankle.y, self.latest_pose.right_ankle.y)
        height = abs(nose - ank)
        hip_ground = abs(hip - ank) < 80
        nose_low   = abs(nose - hip) < 200
        fast_fall  = self.nose_vel > 100

        # 结合 YOLO 初判
        if yolo_flag and fast_fall and height < 200:
            return 5
        elif yolo_flag and hip_ground:
            return 4
        elif hip_ground and nose_low:
            return 3
        elif hip_ground:
            return 2
        elif height < 400:
            return 1
        else:
            return 0

    def main_loop(self):
        while not rospy.is_shutdown():
            evt = FallEvent()
            evt.header.stamp = rospy.Time.now()

            # 1) YOLO 初步检测
            yolo_flag = False
            if self.latest_image:
                img = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
                results = self.yolo(img)
                # 假设模型第 0 类是“人体跌倒”，置信度>0.3 判定为初步跌倒
                for r in results:
                    for cls, conf in zip(r.boxes.cls, r.boxes.conf):
                        if int(cls) == 0 and conf > 0.3:
                            yolo_flag = True

            # 2) 规则引擎细化等级
            level = self.classify_level(yolo_flag)
            evt.is_fall     = (level > 0)
            evt.fall_level = level

            # 发布
            self.pub_evt.publish(evt)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        DecisionAgent()
    except rospy.ROSInterruptException:
        pass
