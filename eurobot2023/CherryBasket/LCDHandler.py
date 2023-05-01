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

        self.pending_message = ""
        self.pending_color = [0, 0, 0]

    def messageInit(self):
        self.lcd.clear()
        # Set LCD color to red
        self.lcd.color = [100, 0, 0]
        self.lcd.message = "Initializing..."

    def setLCDColor(self, r, g, b):
        # Set LCD color to green
        self.pending_color = [r, g, b]
    
    def setLCDMessage(self, message):
        # self.lcd.clear()
        self.pending_message = message

    def clear(self):
        self.lcd.clear()
    
    def beginMonitoring(self):
        print("beginMonitoring")
        t = Thread(target=LCDHandler.monitor, args=[self])
        t.start()
        t2 = Thread(target=LCDHandler.update, args=[self])
        t2.start()
        print("end beginMonitoring")

    def monitor(lcd):

        # detecter les nouveaux fronts montants
        while True:
            upState_old = lcd.upState
            rightState_old = lcd.rightState
            leftState_old = lcd.leftState
            downState_old = lcd.downState
            stateChanged = lcd.stateChanged

            lcd.leftState = lcd.lcd.left_button
            lcd.downState = lcd.lcd.down_button
            lcd.rightState = lcd.lcd.right_button
            lcd.upState = lcd.lcd.up_button

            stateChanged = ((lcd.leftState & (lcd.leftState != leftState_old)) or
                (lcd.downState & (lcd.downState != downState_old)) or
                (lcd.rightState & (lcd.rightState != rightState_old)) or
                (lcd.upState & (lcd.upState != upState_old)))

            if lcd.stateChanged != stateChanged:
                print("stateChanged ", lcd.upState, " ", lcd.downState, " ", lcd.rightState, " ", lcd.leftState)
                print("from ", upState_old, " ", downState_old, " ", rightState_old, " ", leftState_old)
                lcd.stateChanged = stateChanged

            # print("lcd.leftState", lcd.leftState)

            sleep(0.1)
    
    def update(lcd):
        # update lcd every 2 sec, update color every 0.5 sec
        niter = 0
        while True:
            if niter % 4 == 0:
                if lcd.lcd.message != lcd.pending_message:
                    print("Change message")
                    lcd.lcd.message = lcd.pending_message
            if lcd.lcd.color != lcd.pending_color:
                print("Change color")
                lcd.lcd.color = lcd.pending_color
            niter = niter + 1
            sleep(0.5)

    def resetStateChanged(self):
        self.stateChanged = False
        # self.leftState = False
        # self.downState = False
        # self.rightState = False
        # self.upState = False
