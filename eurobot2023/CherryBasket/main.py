# -*- coding: utf-8 -*-

from time import sleep
import os

import socket
import select


from CherryChounter import *
from LCDHandler import *



os.system("rm raw/*.jpg")
os.system("rm blurred/*.jpg")
os.system("rm masked/*.jpg")
os.system("rm grey/*.jpg")
os.system("rm detected/*.jpg")
os.system("rm filtered/*.jpg")

if __name__ == "__main__":
  
  print("LCDHandler")
  lcd = LCDHandler()
  lcd.messageInit()
  LCDHandler.beginMonitoring(lcd)

  print("CherryCounter")
  cherry_counter = CherryCounter()

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
    lcd.setLCDColor(0, 100, 0)
    
    # Method with picamera
    num_cherries = cherry_counter.count_cherries(iter_idx)
    message = "Iter " + str(iter_idx) + " :\n" + str(num_cherries) + " cherries."
    print(message)

    # Set LCD color to blue
    lcd.setLCDColor(0, 0, 100)
    lcd.setLCDMessage(message)
    iter_idx += 1
    sleep(5)
    
  cherry_counter.shutdown()

  sleep(10)

  # Turn off LCD backlights and clear text
  lcd.setLCDColor(0, 0, 0)
  lcd.clear()