#1/usr/bin/enc python3
import rospy

if __name__ == '__main__':

    rospy.init_node("test")

    rospy.loginfo("Test node has been started.")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()