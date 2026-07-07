#!/usr/bin/env python3
"""
test_camera.py — 测试摄像头模拟节点

当没有真实 USB 摄像头时，发布彩色测试图案，
用于在没有硬件的 VM 环境中测试音视频通话功能。

发布话题: ~image_raw (sensor_msgs/Image, BGR8)
参数:
  ~image_width  (int, default: 640)
  ~image_height (int, default: 480)
  ~framerate    (float, default: 25.0)
"""

import rospy
from sensor_msgs.msg import Image
import numpy as np
import math
import time


class TestCameraNode:
    def __init__(self):
        self.width  = rospy.get_param('~image_width',  640)
        self.height = rospy.get_param('~image_height', 480)
        self.fps    = rospy.get_param('~framerate',    25.0)

        self.pub = rospy.Publisher('~image_raw', Image, queue_size=1)
        self.rate = rospy.Rate(self.fps)
        self.counter = 0
        self.start_time = time.time()

        rospy.loginfo("[test_camera] 测试摄像头已启动: %dx%d @ %.1f fps",
                      self.width, self.height, self.fps)

    def make_frame(self):
        """生成彩色测试图案：彩条 + 移动竖线 + 时间戳"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # ── 彩色竖条 ──
        bar_w = max(1, self.width // 7)
        colors_bgr = [
            (255, 0, 0),     # B
            (0, 255, 0),     # G
            (0, 0, 255),     # R
            (255, 255, 0),   # C
            (0, 255, 255),   # Y
            (255, 0, 255),   # M
            (255, 255, 255), # W
        ]
        for i, color in enumerate(colors_bgr):
            x0 = i * bar_w
            x1 = min((i + 1) * bar_w, self.width)
            img[:, x0:x1] = color

        # ── 移动指示线（正弦波位置，证明不是静态图）──
        indicator_x = int((self.width / 2) * (1.0 + math.sin(self.counter * 0.05)))
        indicator_x = max(1, min(indicator_x, self.width - 1))
        img[:, max(0, indicator_x - 2):min(self.width, indicator_x + 3)] = (0, 0, 0)

        # ── 帧计数 ──
        elapsed = time.time() - self.start_time
        actual_fps = self.counter / max(elapsed, 0.001)
        try:
            import cv2
            cv2.putText(img, f"FRAME: {self.counter:06d}  {actual_fps:.1f} FPS",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.putText(img, f"{self.width}x{self.height}  TEST PATTERN",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            # 角色提示
            role = rospy.get_namespace().rstrip('/')
            cv2.putText(img, f"TOPIC: {role}/image_raw",
                        (10, self.height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        except ImportError:
            pass

        return img

    def run(self):
        while not rospy.is_shutdown():
            frame = self.make_frame()

            msg = Image()
            msg.header.stamp    = rospy.Time.now()
            msg.header.frame_id = "test_camera"
            msg.height          = self.height
            msg.width           = self.width
            msg.encoding        = "bgr8"
            msg.is_bigendian    = 0
            msg.step            = self.width * 3
            msg.data            = frame.tobytes()

            self.pub.publish(msg)
            self.counter += 1
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('test_camera')
    node = TestCameraNode()
    node.run()
