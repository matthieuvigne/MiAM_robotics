# -*- coding: utf-8 -*-

import cv2
import numpy as np

def local_minimum(img, x, y):

  kernel_demi_width = 5

  # Window of size 2*kernel_demi_width + 1
  sliced_img = img[
    max(0, x - kernel_demi_width):min(img.shape[0]-1, x + kernel_demi_width), 
    max(0, y - kernel_demi_width):min(img.shape[1]-1, y + kernel_demi_width)]

  localmin = np.quantile(sliced_img, 0.1)

  return localmin

def count_cherries(img, iter_idx):

  cv2.imwrite("raw/image"+str(iter_idx)+"_raw.jpg", img);

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
  cv2.imwrite("detected/image{}_detected.jpg".format(iter_idx), dimg)
  
  num_cherries = len(detected_circles)
  return num_cherries
