import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import time

# Define the Reset Pin
oled_reset = digitalio.DigitalInOut(board.D4)

# Change these
# to the right size for your display!
WIDTH = 128
HEIGHT = 32  # Change to 64 if needed
BORDER = 5

# Use for I2C.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)

# Clear display.
oled.fill(0)
oled.show()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new("1", (oled.width, oled.height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Load default font.
font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 35, encoding="unic")


i = 0

while(i < 1000):

	# Draw a black background
	draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)


	# Draw Some Text
	text = str(i % 100)
	print(text)
	(font_width, font_height) = font.getsize(text)
	draw.text(
	    (oled.width // 2 - font_width // 2, oled.height // 2 - font_height // 2 - 5),
	    text,
	    font=font,
	    fill=255,
	)

	# Display image
	oled.image(image)
	oled.show()
	
	i += 1
	
	time.sleep(1)
