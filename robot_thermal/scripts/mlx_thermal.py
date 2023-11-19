import time
import board
import busio
import numpy as np
import adafruit_mlx90640
import rospy
from std_msgs.msg import String

rospy.init_node('thermal_frame_pub', anonymous=True)
pub = rospy.Publisher('thermal_frame', String, queue_size=1)

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)  # Setup I2C
mlx = adafruit_mlx90640.MLX90640(i2c)  # Initialize MLX90640 with I2C communication
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ  # Set refresh rate
mlx_shape = (24, 32)  # MLX90640 shape

frame = np.zeros(mlx_shape[0] * mlx_shape[1])  # 768 points


def plot_update():
    mlx.getFrame(frame)  # Read MLX90640
    data = np.fliplr(np.reshape(frame,mlx_shape))
    data = str(data)
    # Publish the thermal image
    pub.publish(data)

t_array = []
while not rospy.is_shutdown():
    t1 = time.monotonic()  # For determining frame rate
    try:
        plot_update()  # Update plot
    except:
        continue
    # Approximating frame rate
    t_array.append(time.monotonic() - t1)
    if len(t_array) > 10:
        t_array = t_array[1:]  # Recent times for frame rate approximation
    print('Frame Rate: {0:2.1f}fps'.format(len(t_array) / np.sum(t_array)))

rospy.spin()
