import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from threading import Thread
from time import sleep

class LCDHandler():

    def __init__(self):

        # Modify this if you have a different sized Character LCD
        lcd_columns = 16
        lcd_rows = 2

        # Initialise I2C bus.
        i2c = board.I2C()  # uses board.SCL and board.SDA

        # Initialise the LCD class
        self.lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

        # Initialize button measurements
        self.upState = False
        self.rightState = False
        self.leftState = False
        self.downState = False
        self.stateChanged = False

    def messageInit(self):
        self.lcd.clear()
        # Set LCD color to red
        self.lcd.color = [100, 0, 0]
        self.lcd.message = "Initializing..."

    def setLCDColor(self, r, g, b):
        # Set LCD color to green
        self.lcd.color = [r, g, b]
    
    def setLCDMessage(self, message):
        # self.lcd.clear()
        self.lcd.message = message

    def clear(self):
        self.lcd.clear()
    
    def beginMonitoring(self):
        t = Thread(target=LCDHandler.monitor, args=[self])
        t.start()

    def monitor(lcd):

        # detecter les nouveaux fronts montants
        while True:
            upState_old = lcd.upState
            rightState_old = lcd.rightState
            leftState_old = lcd.leftState
            downState_old = lcd.downState
            stateChanged = lcd.stateChanged

            lcd.leftState = lcd.leftState or lcd.lcd.left_button
            stateChanged = stateChanged or (lcd.leftState & (lcd.leftState != leftState_old))
            lcd.downState = lcd.downState or lcd.lcd.down_button
            stateChanged = stateChanged or (lcd.downState & (lcd.downState != downState_old))
            lcd.rightState = lcd.rightState or lcd.lcd.right_button
            stateChanged = stateChanged or (lcd.rightState & (lcd.rightState != rightState_old))
            lcd.upState = lcd.upState or lcd.lcd.up_button
            stateChanged = stateChanged or (lcd.upState & (lcd.upState != upState_old))

            if lcd.stateChanged != stateChanged:
                print("stateChanged ", lcd.upState, " ", lcd.downState, " ", lcd.rightState, " ", lcd.leftState)

            lcd.stateChanged = stateChanged

            # print("lcd.leftState", lcd.leftState)


            sleep(0.05)

    def resetStateChanged(self):
        self.stateChanged = False
        self.leftState = False
        self.downState = False
        self.rightState = False
        self.upState = False