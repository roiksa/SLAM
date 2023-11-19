import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def receive_thermal_image(data):
    bridge = CvBridge()
    
    # Konversi pesan gambar ROS kembali ke format yang dapat digunakan oleh OpenCV
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    
    # Tampilkan gambar yang diterima
    cv2.imshow("Received Thermal Image", cv_image)
    cv2.waitKey(1)  # Tahan window gambar terbuka

def thermal_image_listener():
    rospy.init_node('thermal_image_listener', anonymous=True)
    rospy.Subscriber('thermal_image', Image, receive_thermal_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        thermal_image_listener()
    except rospy.ROSInterruptException:
        pass
