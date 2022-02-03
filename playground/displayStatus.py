import time
import os
import subprocess
from board import SCL, SDA
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306


# Create the I2C interface.
i2c = busio.I2C(SCL, SDA)

# Create the SSD1306 OLED class.
# The first two parameters are the pixel width and pixel height.  Change these
# to the right size for your display!
disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

# Clear display.
disp.fill(0)
disp.show()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new("1", (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height - padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0


# Load default font.
#font = ImageFont.load_default()
font = ImageFont.truetype("/home/ubuntu/chickies-robot/playground/VCR_OSD_MONO_1.001.ttf", 14)

n = True
while n:
    try:
        #x =  subprocess.check_output("pgrep -x roscore", shell=True)#.stdout
        x = os.system('pgrep -x roscore')
        #x = subprocess.check_output('pgrep -x roscore', shell=True)
        #print(x)
        cmd = "hostname -I | cut -d' ' -f1"
        IP = subprocess.check_output(cmd, shell=True).decode("utf-8")
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        CPU = subprocess.check_output(cmd, shell=True).decode("utf-8")
        #cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%s MB  %.2f%%\", $3,$2,$3*100/$2 }'"
        #MemUsage = subprocess.check_output(cmd, shell=True).decode("utf-8")
        #cmd = 'df -h | awk \'$NF=="/"{printf "Disk: %d/%d GB  %s", $3,$2,$5}\''
        #Disk = subprocess.check_output(cmd, shell=True).decode("utf-8")
        cmd = "cat /sys/class/thermal/thermal_zone0/temp |  awk '{printf \"CPU Temp: %.1f C\", $(NF-0) / 1000}'"  # pylint: disable=line-too-long
        Temp = subprocess.check_output(cmd, shell=True).decode("utf-8")
        #print(IP)
        #print(CPU)
        #print(MemUsage)
        #print(Disk)
        #print(Temp)
        roscoreStatus = ''
        if(x == 0):
            roscoreStatus = 'ROSCORE ACTIVE'
        else:
            roscoreStatus = 'ROSCORE SHUTDOWN'

        # Clear display.
        disp.fill(0)
        #disp.show()
        #draw.text((x, top + 0), 'IP Address:', font=font, fill=255)
        draw.text((x, top + 0), IP, font=font, fill=255)
        #draw.text((x, top + 8), CPU, font=font, fill=255)
        #draw.text((x, top + 16), Temp, font=font, fill=255)
        draw.text((x, top + 16), roscoreStatus, font=font, fill=255)

        # Display image.
        disp.image(image)
        disp.show()
        time.sleep(1)
        #n = False
    except(KeyboardInterrupt):
        n = False