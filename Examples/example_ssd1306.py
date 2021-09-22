import can
from can_Serial import *
from can_I2C import *
from can_utils import *

import time
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import subprocess

import math
import board
import busio

bus = can.Bus(bustype='socketcan',channel='can0',bitrate=50000)
n = CAN_Network_stm32(bus)
listener = n.start()

#ser = CanSerial(canlistener=listener,port="001",baudrate=9600)

# Create the SSD1306 OLED class.
# The first two parameters are the pixel width and pixel height.  Change these
# to the right size for your display!

i2c = I2C_CAN(listener=listener, port="101", frequency=400000, chunkify=16)
#i2c = busio.I2C(board.SCL, board.SDA)



disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height

# Clear display.
disp.fill(0)
disp.show()
# Create image buffer.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new('1', (width, height))

# Load default font.
font = ImageFont.load_default()

# Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as this python script!
# Some nice fonts to try: http://www.dafont.com/bitmap.php
# font = ImageFont.truetype('Minecraftia.ttf', 8)

# Create drawing object.
draw = ImageDraw.Draw(image)

# Define text and get total width.
text = 'SSD1306 ORGANIC LED DISPLAY. THIS IS AN OLD SCHOOL DEMO SCROLLER!! GREETZ TO: LADYADA & THE ADAFRUIT CREW, TRIXTER, FUTURE CREW, AND FARBRAUSCH'
maxwidth, unused = draw.textsize(text, font=font)

# Set animation and sine wave parameters.
amplitude = height/4
offset = height/2 - 4
velocity = -2
startpos = width

# Animate text moving in sine wave.
print('Press Ctrl-C to quit.')
pos = startpos
while True:
    # Clear image buffer by drawing a black filled box.
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    # Enumerate characters and draw them offset vertically based on a sine wave.
    x = pos
    for i, c in enumerate(text):
        # Stop drawing if off the right side of screen.
        if x > width:
            break
        # Calculate width but skip drawing if off the left side of screen.
        if x < -10:
            char_width, char_height = draw.textsize(c, font=font)
            x += char_width
            continue
        # Calculate offset from sine wave.
        y = offset+math.floor(amplitude*math.sin(x/float(width)*2.0*math.pi))
        # Draw text.
        draw.text((x, y), c, font=font, fill=255)
        # Increment x position based on chacacter width.
        char_width, char_height = draw.textsize(c, font=font)
        x += char_width
    # Draw the image buffer.
    disp.image(image)
    disp.show()
    # Move position for next frame.
    pos += velocity
    # Start over if text has scrolled completely off left side of screen.
    if pos < -maxwidth:
        pos = startpos