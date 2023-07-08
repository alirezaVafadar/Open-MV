import sensor, image, time, math, pyb, ustruct

GRAYSCALE_THRESHOLD = (255, 255)
BINARY_VIEW = True
# Initialize I2C communication as slave
# Note: The I2C hardware bus on OpenMV Cam is bus 2.
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
bus.deinit()
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
print("Waiting for Arduino...")

# Set up camera
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # Change pixel format to Grayscale for better contrast and line detection
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)
clock = time.clock()

# Set thresholds for lines and obstacles (based on color)
THRESHOLD_BLACK = (0, 50)
THRESHOLD_WHITE = (200, 255)

# ROI settings
ROI_TOP = 0.5    # Percentage of image height from top
ROI_BOTTOM = 1.0 # Percentage of image height from bottom

# PID control parameters
KP = 0.5
KI = 0
KD = 0

# Initialize errors for PID

old_time = pyb.millis()

while(True):
    left_line = None
    right_line = None
    center_line = None
    clock.tick()
    img = sensor.snapshot()
    if BINARY_VIEW:
        img.binary([GRAYSCALE_THRESHOLD])
        img.erode(1)
    line = img.get_regression([(255,255) if BINARY_VIEW else THRESHOLD])

    if (line): img.draw_line(line.line(), color = 127)
    print("FPS %f, mag = %s" % (clock.fps(), str(line.magnitude()) if (line) else "N/A"))


    # Apply color inversion (optional: uncomment if needed)
    img.invert()

    time.sleep(0.1)  # add delay for stability
