# -*- coding: utf-8 -*-

import cv2
import numpy as np
import picamera
from fractions import Fraction
from picamera import mmal, mmalobj, exc
from picamera.mmalobj import to_rational
from threading import Thread
from time import sleep

RESOLUTION_X = 800
RESOLUTION_Y = 608
RESOLUTION_CHANNELS = 3

RESOLUTION_PARAMETER = (RESOLUTION_X, RESOLUTION_Y)
RESOLUTION_PARAMETER_WITH_CHANNELS = (RESOLUTION_Y, RESOLUTION_X, RESOLUTION_CHANNELS)

MMAL_PARAMETER_ANALOG_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x59
MMAL_PARAMETER_DIGITAL_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x5A

class CherryCounter():
  
  def __init__(self):

    print("init CherryCounter")

    # Setup raspberry
    self.camera = picamera.PiCamera(resolution = RESOLUTION_PARAMETER)
    self.camera.start_preview()

    self.camera.awb_mode = "off"
    self.camera.awb_gains = (Fraction(125, 128), Fraction(579, 256))
    # self.camera.shutter_speed = 1000000 #54036
    self.camera.brightness = 60
    # self.camera.contrast = 70

    self.num_cherries = 0
    self.iter_idx = 0
    self.message = ""

    print("Current a/d gains: {}, {}".format(self.camera.analog_gain, self.camera.digital_gain))

    print("Attempting to set analogue gain to 1")
    set_analog_gain(self.camera, 1)
    print("Attempting to set digital gain to 1")
    set_digital_gain(self.camera, 1)

    print("end init CherryCounter")

  def count_cherries(cherry_counter):
    img = np.empty(RESOLUTION_PARAMETER_WITH_CHANNELS, dtype=np.uint8)


    while True: 

      cherry_counter.iter_idx = (cherry_counter.iter_idx + 1) % 10
      cherry_counter.camera.capture(img, "bgr")
      
      # Common processing
      cv2.imwrite("raw/image"+str(cherry_counter.iter_idx)+"_raw.jpg", img);

      # Threshold the channels
      mask_red = cv2.inRange(img[:,:,2], 110, 255);
      mask_green = cv2.inRange(img[:,:,1],0,90);
      mask_blue = cv2.inRange(img[:,:,0],0,90);
      mask = mask_red*mask_green*mask_blue;
      img = cv2.bitwise_and(img,img, mask=mask);
      #cv2.imwrite("masked/image{}_masked.jpg".format(cherry_counter.iter_idx), img);
      
      # Convert to grey levels
      gimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
      #cv2.imwrite("grey/image{}_grey.jpg".format(cherry_counter.iter_idx), gimg)

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
      
      # Filter : area needs to be filled 70% around circle center
      detected_circles = []
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
      #cv2.imwrite("detected/image{}_detected.jpg".format(cherry_counter.iter_idx), dimg)
      
      cherry_counter.num_cherries = len(detected_circles)
      cherry_counter.message = "Iter " + str(cherry_counter.iter_idx) + " :\n" + str(cherry_counter.num_cherries) + " cherries."
      print(cherry_counter.message)
      
      sleep(1)
  
  def shutdown(self):
    self.camera.stop_preview()
    self.camera.close()

  def beginCountingCherries(self):
    t = Thread(target=CherryCounter.count_cherries, args=[self])
    t.start()
  

def local_minimum(img, x, y):

  kernel_demi_width = 5

  # Window of size 2*kernel_demi_width + 1
  sliced_img = img[
    max(0, x - kernel_demi_width):min(img.shape[0]-1, x + kernel_demi_width), 
    max(0, y - kernel_demi_width):min(img.shape[1]-1, y + kernel_demi_width)]

  localmin = np.quantile(sliced_img, 0.1)

  return localmin

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

