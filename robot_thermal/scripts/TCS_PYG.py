#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

mlx_shape = (24, 32)  # mlx90640 shape
mlx_interp_val = 10  # interpolate # on each dimension
mlx_interp_shape = (mlx_shape[0] * mlx_interp_val,
                    mlx_shape[1] * mlx_interp_val)  # new shape

# Initialize the global variable for the imshow object
heatmap_obj = None

def thermal_frame_callback(data):
    try:
        global heatmap_obj

        print("listening")
        data_array = np.frombuffer(data.data, dtype=np.float32)
        if len(data_array) != mlx_shape[0] * mlx_shape[1]:
            rospy.logwarn("Received data has unexpected size.")
            return

        print("received")
        data_array = np.fliplr(np.reshape(data_array, mlx_shape))
        print("reshaping")
        data_array = ndimage.zoom(data_array, mlx_interp_shape)

        # If imshow object is not initialized, create it
        if heatmap_obj is None:
            plt.ion()  # Turn on interactive mode for non-blocking plot
            plt.figure(figsize=(8, 6))
            heatmap_obj = plt.imshow(data_array, cmap='plasma', interpolation='nearest')
            plt.colorbar()
            plt.title('Heatmap')
        else:
            heatmap_obj.set_array(data_array)  # Update the data

        plt.pause(0.001)  # Non-blocking plot update

    except Exception as e:
        rospy.logerr("Error in thermal_frame_callback: {}".format(str(e)))

def thermal_frame_subscriber():
    rospy.init_node('thermal_frame_subscriber', anonymous=True)
    rospy.Subscriber('/thermal_frame', CompressedImage, thermal_frame_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        thermal_frame_subscriber()
    except rospy.ROSInterruptException:
        pass

