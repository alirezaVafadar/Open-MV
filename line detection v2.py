import sensor, image, time, math
from pyb import I2C

# Initialize I2C communication as slave
i2c = I2C(2, I2C.SLAVE, addr=0x42)

# Set up camera
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # Change pixel format to Grayscale for better contrast and line detection
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)

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
line_position_error = 0
integral_error = 0
last_error = 0
pid_output = 0

def pid_control(line_position_error):
    global integral_error
    global last_error
    integral_error += line_position_error
    derivative_error = line_position_error - last_error
    last_error = line_position_error
    return (KP * line_position_error) + (KI * integral_error) + (KD * derivative_error)

while(True):
    # Initialize variables
    left_line = None
    right_line = None
    center_line = None

    # Get image frame
    img = sensor.snapshot()

    # Apply color inversion (optional: uncomment if needed)
    img.invert()

    # Detect lines
    for l in img.find_lines(roi=(0, int(ROI_TOP*img.height()), img.width(), int(ROI_BOTTOM*img.height())), merge_distance = 30):
        if l.theta() < 45 or l.theta() > 135: # vertical lines
            if not left_line or l.x1() < left_line.x1():
                left_line = l
            if not right_line or l.x2() > right_line.x2():
                right_line = l

    # Detect center line as midpoint of left and right line
    if left_line and right_line:
        center_x = (left_line.x1() + right_line.x1()) // 2
        center_y = (left_line.y1() + right_line.y1()) // 2
        center_line = (center_x, center_y)

    # PID control
    if center_line:
        line_position_error = img.width()//2 - center_line[0]
        pid_output = pid_control(line_position_error)

    # Draw lines
    if left_line: img.draw_line(left_line.line(), color=(0,0,255)) # Blue line for left line
    if right_line: img.draw_line(right_line.line(), color=(255,0,0)) # Red line for right line
    if center_line: img.draw_cross(center_line[0], center_line[1], color=(0,255,0)) # Green cross for center line

    # Send PID output via I2C
    #i2c.send('{:6.2f}'.format(pid_output), timeout=1000)
