# -*- coding: utf-8 -*-

import cv2
from time import sleep
from pathlib import Path
import numpy as np
import subprocess
import picamera
import shutil
import os
from fractions import Fraction

import socket
import select

from set_gain import *

RESOLUTION_X = 800
RESOLUTION_Y = 608
RESOLUTION_CHANNELS = 3

RESOLUTION_PARAMETER = (RESOLUTION_X, RESOLUTION_Y)
RESOLUTION_PARAMETER_WITH_CHANNELS = (RESOLUTION_Y, RESOLUTION_X, RESOLUTION_CHANNELS)

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()
# Set LCD color to red
lcd.color = [100, 0, 0]
lcd.message = "Initializing..."

os.system("rm raw/*.jpg")
os.system("rm blurred/*.jpg")
os.system("rm masked/*.jpg")
os.system("rm grey/*.jpg")
os.system("rm detected/*.jpg")
os.system("rm filtered/*.jpg")


def local_minimum(img, x, y):
  # localmin = 255
  # for i in np.arange(max(0, x - 5), min(img.shape[0]-1, x + 5), 1):
  #   for j in np.arange(max(0, y - 5), min(img.shape[1]-1, y + 5), 1):
  #     # print("img[i, j]) ", img[i, j])
  #     localmin = min(localmin, img[i, j])
  
  sliced_img = img[
    max(0, x - 5):min(img.shape[0]-1, x + 5), 
    max(0, y - 5):min(img.shape[1]-1, y + 5)]
  
  # print(sliced_img.shape)

  localmin = np.quantile(sliced_img, 0.3)

  # print("Local minimum: ", localmin)
  return localmin

def count_cherries(img, iter_idx):

  # Threshold the channels
  mask_red = cv2.inRange(img[:,:,2], 110, 255);
  mask_green = cv2.inRange(img[:,:,1],0,90);
  mask_blue = cv2.inRange(img[:,:,0],0,90);
  mask = mask_red*mask_green*mask_blue;
  img = cv2.bitwise_and(img,img, mask=mask);
  cv2.imwrite("masked/image{}_masked.jpg".format(iter_idx), img);
  
  # Convert to grey levels
  gimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  cv2.imwrite("grey/image{}_grey.jpg".format(iter_idx), gimg)

  # Detect circles using Hough transform
  detected_circles_raw=cv2.HoughCircles(
    gimg, 
    cv2.HOUGH_GRADIENT, 
    1, 
    30,
    param1=50, #100,
    param2 =5, #15, 
    minRadius=15, 
    maxRadius=40
  )
  if detected_circles_raw is None:
    print("number detected circle = 0")
    return 0
  
  # Filter : center of circle needs to be filled
  detected_circles = []
  # kernel1 = np.array([[1, 1, 1],
  #                     [1, 1, 1],
  #                     [1, 1, 1]])
  # # filter2D() function can be used to apply kernel to an image.
  # # Where ddepth is the desired depth of final image. ddepth is -1 if...
  # # ... depth is same as original or source image.
  # fimg = cv2.filter2D(src=gimg, ddepth=-1, kernel=kernel1)
  # cv2.imwrite("filtered/image{}_filtered.jpg".format(iter_idx), fimg)
  for pt in detected_circles_raw[0, :]:
    value = local_minimum(gimg, int(pt[1]), int(pt[0]))
    if value > 10:
      detected_circles.append(pt)
  
  # Image with detected circles
  dimg = cv2.cvtColor(gimg,cv2.COLOR_GRAY2BGR)

  # Draw circles that are detected.  
  for pt in detected_circles:
    pt = np.round(pt).astype(np.int32)
    a, b, r = pt[0], pt[1], pt[2]
    cv2.circle(dimg, (a, b), r, (0, 255, 0), 2)
    cv2.circle(dimg, (a, b), 1, (0, 0, 255), 3)
  cv2.imwrite("detected/image{}_detected.jpg".format(iter_idx), dimg)
  
  num_cherries = len(detected_circles)
  return num_cherries

if __name__ == "__main__":

  # Setup raspberry
  camera = picamera.PiCamera(resolution = RESOLUTION_PARAMETER)
  
  # setup server
  client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
  client.setblocking(0)
  client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

  # Enable broadcasting mode
  client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  client.bind(("192.168.6.255", 37020))

  ### example of udp broadcast using nc
  # client: 
  # echo -n "test data" | nc -u -b 192.168.6.255 37020
  # server:
  # nc -luk 37020

  inputs = [client]
  outputs = []
  message_queues = {}

  sleep(2.0)

  iter_idx = 0;
  max_iters = 100;
  
  # Method with video capture
  # ~ video = cv2.VideoCapture(0);
  # ~ video.set(cv2.CAP_PROP_FRAME_WIDTH,2592);
  # ~ video.set(cv2.CAP_PROP_FRAME_HEIGHT,1944);
  
  # Method with picamera
  camera.start_preview()


  camera.awb_mode = "off"
  camera.awb_gains = (Fraction(125, 128), Fraction(579, 256))
  # camera.shutter_speed = 1000000 #54036
  camera.brightness = 60
  # camera.contrast = 70

  print("Current a/d gains: {}, {}".format(camera.analog_gain, camera.digital_gain))

  print("Attempting to set analogue gain to 1")
  set_analog_gain(camera, 1)
  print("Attempting to set digital gain to 1")
  set_digital_gain(camera, 1)



  
  while(iter_idx<max_iters):
    print("Start iter")

    print("Trying to receive")
    readable, writable, exceptional = select.select(
        inputs, outputs, inputs, 0) # non blocking
    if client in readable:
      print("receiving")
      data, addr = client.recvfrom(1024)
      print("received message: %s" % data)


    # Set LCD color to blue
    lcd.color = [0, 100, 0]

    # Method with raspistill
    # ~ subprocess.call("raspistill -o image{}_raw.jpg".format(iter_idx),shell=True);
    # ~ image_name = "image{}_raw.jpg".format(iter_idx);
    # ~ img = cv2.imread(image_name, cv2.IMREAD_COLOR)
    # ~ width = int(0.4*img.shape[1])
    # ~ height = int(0.4*img.shape[0])
    # ~ img = cv2.resize(img, (width,height), interpolation = cv2.INTER_AREA);
    
    # Method with video capture
    # ~ _, img = video.read();
    # ~ img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB);
    # ~ img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR);
    
    # Method with picamera
    img = np.empty(RESOLUTION_PARAMETER_WITH_CHANNELS, dtype=np.uint8)
    camera.capture(img, "bgr");
    # ~ image_name = "image{}_raw.jpg".format(iter_idx);
    # ~ cv2.imwrite(image_name, img);
    
    # Common processing
    cv2.imwrite("raw/image"+str(iter_idx)+"_raw.jpg", img);
    # cv2.imwrite("image"+str(iter_idx)+"_raw_red.jpg", img[:,:,2]);
    # cv2.imwrite("image"+str(iter_idx)+"_raw_green.jpg", img[:,:,1]);
    # cv2.imwrite("image"+str(iter_idx)+"_raw_blue.jpg", img[:,:,0]);
    # cv2.imwrite("image"+str(iter_idx)+"_raw.jpg", img);
    # cv2.imwrite("image"+str(iter_idx)+"_raw.jpg", img);
    num_cherries = count_cherries(img,iter_idx);
    message = "Iter " + str(iter_idx) + " :\n" + str(num_cherries) + " cherries."
    print(message);

    lcd.clear()

    # Set LCD color to blue
    lcd.color = [0, 0, 100]
    lcd.message = message
    iter_idx += 1;
    sleep(5);
    
  # Method with picamera
  camera.stop_preview();
  camera.close();
  
  # Method with video capture
  # ~ video.release();

  sleep(10);

  # Turn off LCD backlights and clear text
  lcd.color = [0, 0, 0]
  lcd.clear()