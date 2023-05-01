# -*- coding: utf-8 -*-

from time import sleep, time
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
  lcd.beginMonitoring()

  print("CherryCounter")
  cherry_counter = CherryCounter()
  cherry_counter.beginCountingCherries()

  # setup server
  client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
  client.setblocking(0)
  client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

  # Enable broadcasting mode
  client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  client.bind(("", 37020))

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

  matchStarted = False
  matchStartTime = 0
  matchFinished = False
  lastCherryCount = 0


  while(True):

    # print("matchStarted ", matchStarted)

    # Two ways to start match: either receive signal from
    # network or press button
    print("Trying to receive")
    readable, writable, exceptional = select.select(
        inputs, outputs, inputs, 0) # non blocking
    if client in readable:
      print("receiving")
      data, addr = client.recvfrom(1024)
      print("received message: %s" % data)
      matchStarted = True
      matchStartTime = time()
      matchFinished = False

    if lcd.stateChanged:
      matchStarted = not matchStarted
      matchFinished = False
      matchStartTime = time()
      lcd.resetStateChanged()
      print("Reset match")

    if not matchStarted:
      # Set LCD color to blue
      lcd.setLCDColor(0, 0, 100)
      lcd.setLCDMessage("Wait for start".rjust(16, " ") + "\n" + (str(cherry_counter.num_cherries) + " cherries").rjust(16, " "))
    elif matchStarted and not matchFinished:
    # Set LCD color to green
      lcd.setLCDColor(0, 100, 0)
      lcd.setLCDMessage(("Time " + str(round(time() - matchStartTime))).rjust(16, " ") + "\n" + (str(cherry_counter.num_cherries) + " cherries").rjust(16, " "))
      lastCherryCount = cherry_counter.num_cherries

    if matchStarted and not matchFinished:
      if (time() - matchStartTime) > 100:
        print("End match")
        matchFinished = True
        lcd.setLCDColor(50, 0, 50)
        lcd.setLCDMessage("Match finished".rjust(16, " ") + "\n" + (str(lastCherryCount) + " cherries").rjust(16, " "))
    
    sleep(0.2)

    
  cherry_counter.shutdown()

  sleep(10)

  # Turn off LCD backlights and clear text
  lcd.setLCDColor(0, 0, 0)
  lcd.clear()
