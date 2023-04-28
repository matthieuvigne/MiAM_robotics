# -*- coding: utf-8 -*-

import cv2
from time import sleep
from pathlib import Path
import numpy as np
import subprocess
import picamera
import shutil
import os

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

os.system("rm *.jpg")

# Setup raspberry
camera = picamera.PiCamera(resolution =(2592, 1952), framerate = 15)

def count_cherries(img,iter_idx):

  # Threshold the red channel
  img = cv2.blur(img, (15,15));
  cv2.imwrite("image{}_blurred.jpg".format(iter_idx), img);
  mask_red = cv2.inRange(img[:,:,2], 150, 255);
  mask_green = cv2.inRange(img[:,:,1],0,110);
  mask_blue = cv2.inRange(img[:,:,0],0,110);
  mask = mask_red*mask_green*mask_blue;
  img = cv2.bitwise_and(img,img, mask=mask);
  cv2.imwrite("image{}_filtered.jpg".format(iter_idx), img);

  # Detect the blobs and count them
  img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  retval, _, stats, _	=	cv2.connectedComponentsWithStatsWithAlgorithm( \
    img, 8, cv2. CV_32S, cv2.CCL_WU);
  num_cherries = 0;
  for stat in stats[1:]:
    width = stat[2];
    height = stat[3];
    ok = np.abs(np.log2(width/height))<1.0;
    ok &= min(width,height)>30;
    if ok:
      num_cherries += 1;
  return num_cherries;
  # ~ return retval-1;

if __name__ == "__main__":

  iter_idx = 0;
  max_iters = 10;
  
  # Method with video capture
  # ~ video = cv2.VideoCapture(0);
  # ~ video.set(cv2.CAP_PROP_FRAME_WIDTH,2592);
  # ~ video.set(cv2.CAP_PROP_FRAME_HEIGHT,1944);
  
  # Method with picamera
  camera.start_preview();


  
  while(iter_idx<max_iters):
    print("Start iter")

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
    img = np.empty((1952, 2592, 3), dtype=np.uint8)
    camera.capture(img, "bgr");
    # ~ image_name = "image{}_raw.jpg".format(iter_idx);
    # ~ cv2.imwrite(image_name, img);
    
    # Common processing
    cv2.imwrite("image"+str(iter_idx)+"_raw.jpg", img);
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