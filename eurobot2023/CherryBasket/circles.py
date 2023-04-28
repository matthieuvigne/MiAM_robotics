# -*- coding: utf-8 -*-

import cv2
import numpy as np

for idx in range(1,9):

  # Read, resize and show the image
  image_name = "image" +  str(idx) + ".jpg";
  img = cv2.imread(image_name, cv2.IMREAD_COLOR)
  width = int(0.4*img.shape[1])
  height = int(0.4*img.shape[0])
  img = cv2.resize(img, (width,height), interpolation = cv2.INTER_AREA)
  cv2.imshow("Original image", img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  
  # Threshold the red channel
  img = cv2.blur(img, (15,15))
  mask_red = cv2.inRange(img[:,:,2], 230, 255)
  mask_green = cv2.inRange(img[:,:,1],0,160);
  mask_blue = cv2.inRange(img[:,:,0],0,160);
  mask = mask_red*mask_green*mask_blue;
  img = cv2.bitwise_and(img,img, mask=mask)
  cv2.imshow("Most red pixels", img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()

  # Detect the blobs and count them
  img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  retval, _, stats, _	=	cv2.connectedComponentsWithStatsWithAlgorithm( \
    img, 8, cv2. CV_32S, cv2.CCL_WU)
  for stat in stats[1:]:
    width = stat[2];
    height = stat[3];
    area = width*height;
    print("(Width,Height) = ({},{})".format(width,height));
    print("Area = {} px^2".format(area));
  print(retval-1)
