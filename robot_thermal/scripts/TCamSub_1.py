#!/usr/bin/env python
import rospy
import json
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Byte, String
import numpy as np
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage

mlx_shape = (24,32) # mlx90640 shape
mlx_interp_val = 10 # interpolate # on each dimension
mlx_interp_shape = (mlx_shape[0]*mlx_interp_val,
                    mlx_shape[1]*mlx_interp_val) # new shape
fig = plt.figure(figsize=(12,9)) # start figure
ax = fig.add_subplot(111) # add subplot
fig.subplots_adjust(0.05,0.05,0.95,0.95) # get rid of unnecessary padding
therm1 = ax.imshow(np.zeros(mlx_interp_shape),interpolation='none',
                   cmap=plt.cm.plasma,vmin=25,vmax=45) # preemptive image
cbar = fig.colorbar(therm1) # setup colorbar
cbar.set_label('Temperature [$^{\circ}$C]',fontsize=14) # colorbar label

fig.canvas.draw() # draw figure to copy background
ax_background = fig.canvas.copy_from_bbox(ax.bbox) # copy background
fig.show() # show the figure before blitting
frame = np.zeros(mlx_shape[0]*mlx_shape[1]) # 768 pts

def thermal_frame_callback(data): 
    fig.canvas.restore_region(ax_background) # restore background
    print("listening")
    frame = np.frombuffer(data.data, dtype=np.float32)
    print("received")
    return

def plot_update():
    frame = ndimage.zoom(frame, mlx_interp_val)
    print(frame)
    therm1.set_array(frame)  # Set data
    therm1.set_clim(vmin=np.min(frame), vmax=np.max(frame))  # Set bounds for the image
    cbar.update_normal(therm1)    # Update the colorbar based on the new data range
    print("Color bar updated")
    ax.draw_artist(therm1)  # Draw new thermal image
    print("update draw")
    fig.canvas.blit(ax.bbox)  # Draw background
    print("Update background")
    fig.canvas.flush_events()  # Show the new image
    print("show image")

    return

def thermal_frame_subscriber():
    rospy.init_node('thermal_frame_subscriber', anonymous=True)
    rospy.Subscriber('/thermal_frame', CompressedImage, thermal_frame_callback)    
    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        thermal_frame_subscriber()
        plot_update()
    except rospy.ROSInterruptException:
        pass
