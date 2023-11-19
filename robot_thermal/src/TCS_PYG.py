#!/usr/bin/env python
import rospy
import json
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Byte, String
import numpy as np
import pygame

mlx_shape = (24,32) # mlx90640 shape
mlx_interp_val = 10 # interpolate # on each dimension
mlx_interp_shape = (mlx_shape[0]*mlx_interp_val,
                    mlx_shape[1]*mlx_interp_val) # new shape
# Callback function for receiving array data
# define screen size
SCREEN_WIDTH = 240
SCREEN_HEIGHT = 320

# define colours
pink = [255,0,255]
purple = [191,0,255]
blue = [0,0,255]
light_blue = [0,102,255]
very_light_blue = [153,204,255]
white = [255,255,255]
light_grey = [217,217,217]
red = [255,0,0]
dark_red = [179,0,0]
black = [0,0,0]
light_green = [204, 255,204]
green = [0,255,0]
dark_green = [0,128,0]

#set up pygame
size = [SCREEN_WIDTH, SCREEN_HEIGHT]
# set up the screen and its size
screen = pygame.display.set_mode(size)
pygame.init()
screen.fill(black)
pygame.mouse.set_visible(False)

# print temps and colours to screen
font = pygame.font.SysFont(None, 30)
temp1 = font.render('<27', True, white)
temp2 = font.render('29', True, light_grey)
temp3 = font.render('31', True, very_light_blue)
temp4 = font.render('33', True, blue)
temp5 = font.render('35', True, purple)
temp6 = font.render('37', True, pink)
temp7 = font.render('39', True, red)
temp8 = font.render('40>', True, dark_red)
temp9 = font.render('<0', True, dark_green)
temp10 = font.render('<5', True, green)
temp11 = font.render('<10', True, light_green)
screen.blit(temp1, (10, 180))
screen.blit(temp2, (50, 180))
screen.blit(temp3, (80, 180))
screen.blit(temp4, (110, 180))
screen.blit(temp5, (140, 180))
screen.blit(temp6, (170, 180))
screen.blit(temp7, (200, 180))
screen.blit(temp8, (230, 180))
screen.blit(temp9, (10, 205))
screen.blit(temp10, (50, 205))
screen.blit(temp11, (90, 205))

# ghost advice text
font = pygame.font.SysFont(None, 30)
adv1 = font.render('Living People: ', True, white)
adv2 = font.render('33 to 37 degrees', True, white)
screen.blit(adv1, (400, 180))
screen.blit(adv2, (400, 205))

# set up the camera, but no preview
camera = PiCamera()
camera.resolution = (214, 160)
camera.framerate = 15
time.sleep(.5)
#define the frame size for the thermal sensor
frame = [0] * 768

def thermal_frame_callback(data):
    print("listening")
    data_array = np.frombuffer(data.data, dtype=np.float32)
    print("received")
    data_array = np.fliplr(np.reshape(data_array, mlx_shape))
    print("reshaping")
    data_array = ndimage.zoom(data_array, mlx_interp_shape)
    print(data_array)
    row = 1
    column = 1
    rect_width = 7
    rect_height = 7
    for row in range(24):
        row_pos = int((row * 10) * 0.67)
        for column in range(32):
            temp = frame[row * 32 + column]
            
            col_pos = 214 - (int((column * 10) * 0.67))
            
            
            if temp < 27:
                pygame.draw.rect(screen, white, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            elif temp < 29:
                pygame.draw.rect(screen, light_grey, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            elif temp < 31:
                pygame.draw.rect(screen, very_light_blue, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            elif temp < 33:
                pygame.draw.rect(screen, blue, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            elif temp < 35:
                pygame.draw.rect(screen, purple, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            elif temp < 37:
                pygame.draw.rect(screen, pink, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            elif temp < 39:
                pygame.draw.rect(screen, red, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            elif temp > 38:
                pygame.draw.rect(screen, dark_red, pygame.Rect(col_pos, row_pos , rect_width, rect_height))
            
            pygame.display.update(pygame.Rect(col_pos,row_pos,7,7))
    pygame.display.flip()

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

