#!/usr/bin/env python
import rospy
import json
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Byte, String
import numpy as np
import time
mlx_shape = (24,32) # mlx90640 shape

mlx_interp_val = 10 # interpolate # on each dimension
mlx_interp_shape = (mlx_shape[0]*mlx_interp_val,
                    mlx_shape[1]*mlx_interp_val) # new shape
                    


def thermal_frame_callback(data):
    fig.canvas.restore_region(ax_background) # restore background
    print("listening")
    frame = np.frombuffer(data.data, dtype=np.float32)
    print("received")
    print(frame.shape)
    data_array = np.fliplr(np.reshape(frame, mlx_shape))
    print("reshaping")
    print(data_array.shape)
    data_array = ndimage.zoom(data_array, mlx_interp_val)
    print(data_array)
    print(data_array.shape)

    return

def thermal_frame_subscriber():
    rospy.init_node('thermal_frame_subscriber', anonymous=True)
    rospy.Subscriber('/thermal_frame', CompressedImage, thermal_frame_callback)    
    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        thermal_frame_subscriber()
    except rospy.ROSInterruptException:
        pass

