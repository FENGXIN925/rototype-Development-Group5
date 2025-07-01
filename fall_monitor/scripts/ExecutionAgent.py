#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from jupiterobot2_msgs.msg import FallEvent
from sound_play.libsoundplay import SoundClient

class ExecutionAgent:
    def __init__(self):
        rospy.init_node('execution_agent', anonymous=True)
        self.sound = SoundClient()
        rospy.Subscriber('/fall_event', FallEvent, self.cb, queue_size=1)
        rospy.loginfo("ExecutionAgent started.")
        rospy.spin()

    def cb(self, msg: FallEvent):
        if not msg.is_fall:
            return
        lvl = msg.fall_level
        text = f"检测到 {lvl} 级跌倒。"
        rospy.loginfo(text)
        # English TTS
        self.sound.say(f"Fall level {lvl} detected.")
        # 中文 TTS（根据你的音库支持）
        # self.sound.say(f"检测到 {lvl} 级跌倒。", lang='zh')

if __name__ == '__main__':
    try:
        ExecutionAgent()
    except rospy.ROSInterruptException:
        pass
