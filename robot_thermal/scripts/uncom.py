#!/usr/bin/python3

import cv2
import numpy as np
import rospy
import time
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class ImageViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_subscriber = rospy.Subscriber(
            '/image', CompressedImage, self.process_img_msg
        )
        rospy.loginfo("Subscribed to /image, ready to show image")
        self.img_publisher = rospy.Publisher('/image_raw', Image, queue_size = 1)
        self.id = 0
        

    def process_img_msg(self, cImg_msg: CompressedImage):
        if self.id == 0:
            self.start = time.time()
        if self.id == 6000:
            self.start = time.time()
            self.id = 0
        self.id+=1
        """ callback function for publisher """
        np_img = np.fromstring(cImg_msg.data, np.uint8)
        img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        # self.imgMsg.header.stamp = cImg_msg.header.stamp
        # self.imgMsg.data = img
        img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        img_msg.header.stamp = cImg_msg.header.stamp
        print('fps: ',self.id/ (time.time()- self.start ))
        cv2.imshow('frame', img)
        self.img_publisher.publish(img_msg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.loginfo("Shutting Down...")
            rospy.signal_shutdown('User Interrupt with Q')

if __name__ == "__main__":
    rospy.init_node("ImageViewer_node")
    Viewer = ImageViewer()
    rospy.spin()
