#!/usr/bin/env python
import rospy
from std_msgs.msg import String
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
                   cmap=plt.cm.bwr,vmin=25,vmax=45) # preemptive image
cbar = fig.colorbar(therm1) # setup colorbar
cbar.set_label('Temperature [$^{\circ}$C]',fontsize=14) # colorbar label

fig.canvas.draw() # draw figure to copy background
ax_background = fig.canvas.copy_from_bbox(ax.bbox) # copy background
fig.show() # show the figure before blitting

def thermal_frame_callback(data):
    print(type(data))
    fig.canvas.restore_region(ax_background) # restore background
    datafloat = float(data)
    datafloat = np.fliplr(np.reshape(datafloat,mlx_shape))
    datafloat = ndimage.zoom(datafloat,mlx_interp_val) # interpolate
    therm1.set_array(datafloat) # set data
    therm1.set_clim(vmin=np.min(datafloat),vmax=np.max(datafloat)) # set bounds
    cbar.on_mappable_changed(therm1) # update colorbar range
    ax.draw_artist(therm1) # draw new thermal image
    fig.canvas.blit(ax.bbox) # draw background
    fig.canvas.flush_events() # show the new image
    return

def thermal_frame_subscriber():
    rospy.init_node('thermal_frame_subscriber', anonymous=True)
    rospy.Subscriber('thermal_frame', String, thermal_frame_callback)
    
    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        thermal_frame_subscriber()
    except rospy.ROSInterruptException:
        pass
