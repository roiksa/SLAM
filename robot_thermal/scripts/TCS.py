#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from scipy import ndimage
from cv_bridge import CvBridge

mlx_shape = (24, 32)  # mlx90640 shape
mlx_interp_val = 1  # interpolate # on each dimension
mlx_interp_shape = (mlx_shape[0] * mlx_interp_val,
                    mlx_shape[1] * mlx_interp_val)  # new shape

# Initialize the global variable for the imshow object
heatmap_obj = None

def thermal_frame_callback(data):
    global heatmap_obj

    try:
        print("listening")
        data_array = np.frombuffer(data.data, dtype=np.float32)
        if len(data_array) != mlx_shape[0] * mlx_shape[1]:
            rospy.logwarn("Received data has unexpected size.")
            return

        print("received")
        data_array = np.fliplr(np.reshape(data_array, mlx_shape))
        print("reshaping")
        data_array = ndimage.zoom(data_array, mlx_interp_shape,order = 3 )
        print(data_array.max()) 
        cmap = cm.plasma((data_array-data_array.min())/(data_array.max()-data_array.min()))
        TImg = cmap[:,:,0:3]*255
        TImg = TImg.astype(np.uint8)
        #print(TImg[124,124])
        #print(TImg.shape)
        #data_array_normalized = (data_array-25)/(45-25)*255
        #TImg = data_array_normalized.astype(np.uint8)
        #TImg = np.dstack((TImg,TImg,TImg))
        #print(TImg[124,124])
        #print(TImg.shape)
        
        
        
        # If imshow object is not initialized, create it
        if heatmap_obj is None:
            plt.ion()  # Turn on interactive mode for non-blocking plot
            plt.figure(figsize=(8, 6))
            heatmap_obj = plt.imshow(TImg)
            #heatmap_obj = plt.imshow(data_array, cmap='plasma', interpolation="nearest")
            plt.colorbar()
            plt.title('Heatmap')
        else:
            heatmap_obj.set_array(TImg)  # Update the data
    
        plt.pause(0.001)  # Non-blocking plot update

        # Publish the thermal image
        bridge = CvBridge()
        #image_msg = bridge.cv2_to_imgmsg(np.uint8(data_array * 255), encoding="passthrough")
        image_msg = bridge.cv2_to_imgmsg(TImg, encoding="rgb8")
        image_pub.publish(image_msg)
        print("img publish")

    except Exception as e:
        rospy.logerr("Error in thermal_frame_callback: {}".format(str(e)))

def thermal_frame_subscriber():
    rospy.Subscriber('/thermal_frame', CompressedImage, thermal_frame_callback)
    print ("subscrived")
    rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize the ROS node only once
        rospy.init_node('thermal_publisher', anonymous=True)
        
        # Initialize the image publisher
        image_pub = rospy.Publisher('/thermal_img_topic', Image, queue_size=10)
        
        # Start the thermal frame subscriber
        thermal_frame_subscriber()

        # Close the plot window when the node is shutdown
        plt.ioff()
        plt.show()

    except rospy.ROSInterruptException:
        pass

