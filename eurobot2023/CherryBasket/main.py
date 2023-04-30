# -*- coding: utf-8 -*-

from time import sleep
import picamera
import os

import socket
import select

from fractions import Fraction

from set_gain import *
from count_cherries import *


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

  # wait for init
  sleep(2.0)

  ### example of udp broadcast using nc
  # client: 
  # echo -n "test data" | nc -u -b 192.168.6.255 37020
  # server:
  # nc -luk 37020

  inputs = [client]
  outputs = []
  message_queues = {}

  iter_idx = 0
  max_iters = 100
  
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

    # Set LCD color to green
    lcd.color = [0, 100, 0]
    
    # Method with picamera
    img = np.empty(RESOLUTION_PARAMETER_WITH_CHANNELS, dtype=np.uint8)
    camera.capture(img, "bgr")
    
    # Common processing
    num_cherries = count_cherries(img,iter_idx)
    message = "Iter " + str(iter_idx) + " :\n" + str(num_cherries) + " cherries."
    print(message)

    lcd.clear()

    # Set LCD color to blue
    lcd.color = [0, 0, 100]
    lcd.message = message
    iter_idx += 1
    sleep(5)
    
  # Method with picamera
  camera.stop_preview()
  camera.close()

  sleep(10)

  # Turn off LCD backlights and clear text
  lcd.color = [0, 0, 0]
  lcd.clear()