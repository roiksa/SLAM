import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time,board,busio
import adafruit_mlx90640
import matplotlib.pyplot as plt
from scipy import ndimage

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000) # setup I2C
mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ # set refresh rate
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
# fig.show() # show the figure before blitting
frame = np.zeros(mlx_shape[0]*mlx_shape[1]) # 768 pts

rospy.init_node('thermal_camera_publisher')
thermal_image_pub = rospy.Publisher('thermal_image', Image, queue_size=10)
bridge = CvBridge()



def publish_thermal_image():
    fig.canvas.restore_region(ax_background)
    mlx.getFrame(frame)
    data_array = np.fliplr(np.reshape(frame, mlx_shape))
    data_array = ndimage.zoom(data_array, mlx_interp_val)
    therm1.set_array(data_array)
    therm1.set_clim(vmin=np.min(data_array), vmax=np.max(data_array))
    cbar.on_mappable_changed(therm1)

    ax.draw_artist(therm1)
    fig.canvas.blit(ax.bbox)
    fig.canvas.flush_events()

    # Convert the data_array to an 8-bit representation
    data_array_normalized = ((data_array - 25) / (45 - 25)) * 255
    data_array_uint8 = data_array_normalized.astype(np.uint8)

    # Convert the array to an image that can be handled by CvBridge
    image_message = bridge.cv2_to_imgmsg(data_array_uint8, encoding="mono8")

    # Publish the thermal image to the "thermal_image" topic
    thermal_image_pub.publish(image_message)
    
if __name__ == '__main__':
    try:
        rate = rospy.Rate(2)  # Frekuensi gambar Anda
        while not rospy.is_shutdown():
            publish_thermal_image()  # Kirim data gambar termal ke ROS
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
