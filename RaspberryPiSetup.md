# Rotate screen

Add to /boot/config.txt:

display_lcd_rotate=2
lcd_rotate=2

First line for screen display, second for touchscreen coordinates ; 2 = 180deg

# Prevent screensaver

Make sure not to use ssh -X, else the wrong xserver will be used (i.e. screensaver on PC not on raspi)

pi@raspberrypi:~ $ DISPLAY=:0 xset s 0 0

To check: DISPLAY=:0 xset q should show

Screen Saver:
  prefer blanking:  no    allow exposures:  no
  timeout:  0    cycle:  0


# 400kHz I2C
Add to /boot/config.txt:

dtparam=i2c_arm=on,i2c_arm_baudrate=400000


# Getting Edimax EW-7811Un WiFi doongle to work

Just make sure the rtl8xxxu driver is not blacklisted in /etc/modprobe.d/b
