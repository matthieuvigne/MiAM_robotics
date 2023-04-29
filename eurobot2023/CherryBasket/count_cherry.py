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

from picamera import mmal, mmalobj, exc
from picamera.mmalobj import to_rational

MMAL_PARAMETER_ANALOG_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x59
MMAL_PARAMETER_DIGITAL_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x5A

RESOLUTION_X = 800
RESOLUTION_Y = 608
RESOLUTION_CHANNELS = 3

RESOLUTION_PARAMETER = (RESOLUTION_X, RESOLUTION_Y)
RESOLUTION_PARAMETER_WITH_CHANNELS = (RESOLUTION_Y, RESOLUTION_X, RESOLUTION_CHANNELS)


def set_gain(camera, gain, value):
    """Set the analog gain of a PiCamera.
    
    camera: the picamera.PiCamera() instance you are configuring
    gain: either MMAL_PARAMETER_ANALOG_GAIN or MMAL_PARAMETER_DIGITAL_GAIN
    value: a numeric value that can be converted to a rational number.
    """
    if gain not in [MMAL_PARAMETER_ANALOG_GAIN, MMAL_PARAMETER_DIGITAL_GAIN]:
        raise ValueError("The gain parameter was not valid")
    ret = mmal.mmal_port_parameter_set_rational(camera._camera.control._port, 
                                                    gain,
                                                    to_rational(value))
    if ret == 4:
        raise exc.PiCameraMMALError(ret, "Are you running the latest version of the userland libraries? Gain setting was introduced in late 2017.")
    elif ret != 0:
        raise exc.PiCameraMMALError(ret)

def set_analog_gain(camera, value):
    """Set the gain of a PiCamera object to a given value."""
    set_gain(camera, MMAL_PARAMETER_ANALOG_GAIN, value)

def set_digital_gain(camera, value):
    """Set the digital gain of a PiCamera object to a given value."""
    set_gain(camera, MMAL_PARAMETER_DIGITAL_GAIN, value)



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

def count_cherries(img,iter_idx):

  # Threshold the red channel
  # img = cv2.blur(img, (15,15));
  # cv2.imwrite("blurred/image{}_blurred.jpg".format(iter_idx), img);
  mask_red = cv2.inRange(img[:,:,2], 110, 255);
  mask_green = cv2.inRange(img[:,:,1],0,90);
  mask_blue = cv2.inRange(img[:,:,0],0,90);
  mask = mask_red*mask_green*mask_blue;
  img = cv2.bitwise_and(img,img, mask=mask);
  cv2.imwrite("masked/image{}_masked.jpg".format(iter_idx), img);

  # greyscale
  # img = img[:, :, 2] #cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  gimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  cv2.imwrite("grey/image{}_grey.jpg".format(iter_idx), gimg);

  # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(8,8))
  # # gimg = cv2.erode(gimg,kernel,iterations = 1)
  # gimg = cv2.morphologyEx(gimg, cv2.MORPH_OPEN, kernel)
  # gimg = cv2.morphologyEx(gimg, cv2.MORPH_CLOSE, kernel)
  # # gimg = cv2.Canny(gimg,100,200)

  cv2.imwrite("filtered/image{}_filtered.jpg".format(iter_idx), gimg);

  # Detect the blobs and count them
  # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  # retval, _, stats, _	=	cv2.connectedComponentsWithStatsWithAlgorithm( \
  #   img, 8, cv2. CV_32S, cv2.CCL_WU);



  # retval, _, stats, _	=	cv2.connectedComponentsWithStatsWithAlgorithm( \
  #   img, 8, cv2. CV_32S, cv2.CCL_WU);
  # num_cherries = 0;
  # for stat in stats[1:]:
  #   width = stat[2];
  #   height = stat[3];
  #   ok = np.abs(np.log2(width/height))<1.0;
  #   ok &= min(width,height)>30;
  #   if ok:
  #     num_cherries += 1;

  # Apply Hough transform on the blurred image (40, 30, 55) pour ligne cerise frontale
  # detected_circles=cv2.HoughCircles(gimg,cv2.HOUGH_GRADIENT,1,25,param1=50,param2 =30,minRadius=10,maxRadius=30)



  # method	Detection method, see HoughModes. The available methods are HOUGH_GRADIENT and HOUGH_GRADIENT_ALT.
  # dp	Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height. For HOUGH_GRADIENT_ALT the recommended value is dp=1.5, unless some small very circles need to be detected.
  # minDist	Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
  # param1	First method-specific parameter. In case of HOUGH_GRADIENT and HOUGH_GRADIENT_ALT, it is the higher threshold of the two passed to the Canny edge detector (the lower one is twice smaller). Note that HOUGH_GRADIENT_ALT uses Scharr algorithm to compute image derivatives, so the threshold value shough normally be higher, such as 300 or normally exposed and contrasty images.
  # param2	Second method-specific parameter. In case of HOUGH_GRADIENT, it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first. In the case of HOUGH_GRADIENT_ALT algorithm, this is the circle "perfectness" measure. The closer it to 1, the better shaped circles algorithm selects. In most cases 0.9 should be fine. If you want get better detection of small circles, you may decrease it to 0.85, 0.8 or even less. But then also try to limit the search range [minRadius, maxRadius] to avoid many false circles.
  # minRadius	Minimum circle radius.
  # maxRadius	Maximum circle radius. If <= 0, uses the maximum image dimension. If < 0, HOUGH_GRADIENT returns centers without finding the radius. HOUGH_GRADIENT_ALT always computes circle radiuses.

  detected_circles_raw=cv2.HoughCircles(
    gimg, 
    cv2.HOUGH_GRADIENT, 
    1, 
    30,
    param1=50, #100,
    param2 =5, #15, 
    minRadius=5, 
    maxRadius=40
  )

  # params = cv2.SimpleBlobDetector_Params()
  # detector = cv2.SimpleBlobDetector_create(params)
  # detected_circles = detector.detect(gimg)


  # Draw circles that are detected.  
  if detected_circles_raw is None:
    print("number detected circle = 0")
    return 0

  print(detected_circles_raw.shape)
  
  detected_circles = []
  for pt in detected_circles_raw[0, :]:
    value = gimg[int(pt[1]), int(pt[0])]
    if value > 10:
      # print("Detected new circle")
      detected_circles.append(pt)
  
  # print(detected_circles)
  
  dimg = cv2.cvtColor(gimg,cv2.COLOR_GRAY2BGR)

  # print(detected_circles)

  for pt in detected_circles:
    pt = np.round(pt).astype(np.int32)
    # print(pt)
    a, b, r = pt[0], pt[1], pt[2]
    cv2.circle(dimg, (a, b), r, (0, 255, 0), 2)
    cv2.circle(dimg, (a, b), 1, (0, 0, 255), 3)
    #cv2.imshow("Detected Circle", frame) 

  cv2.imwrite("detected/image{}_detected.jpg".format(iter_idx), dimg);
  # # Convert the circle parameters a, b and r to integers.
  # detected_circles= np.uint16(np.around(detected_circles))
  # print(detected_circles)
  # print (len(detected_circles[0, :]))
  num_cherries = len(detected_circles)

  return num_cherries;
  # ~ return retval-1;

if __name__ == "__main__":

  # Setup raspberry
  camera = picamera.PiCamera(resolution = RESOLUTION_PARAMETER)

  sleep(2.0)

  iter_idx = 0;
  max_iters = 10;
  
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

  print("Current a/d gains: {}, {}".format(camera.analog_gain, camera.digital_gain))

  print("Attempting to set analogue gain to 1")
  set_analog_gain(camera, 1)
  print("Attempting to set digital gain to 1")
  set_digital_gain(camera, 1)



  
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