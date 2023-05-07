import sensor, image
from pyb import I2C

# Initialize I2C communication as slave
i2c = I2C(2, I2C.SLAVE, addr=0x42)

# Thresholds for detecting lines (may need to be adjusted based on lighting conditions)
THRESHOLD_BLACK = (0, 50)
THRESHOLD_WHITE = (200, 255)

# ROI (region of interest) for detecting lines
ROI_TOP = 0.5    # Percentage of image height from top
ROI_BOTTOM = 1.0 # Percentage of image height from bottom

# PID control parameters for line following
KP = 0.5
KI = 0
KD = 0

# Initialize line position error and integral error
line_position_error = 0
integral_error = 0
last_error = 0
pid_output = 0

# Define function to calculate PID control output
def pid_control(line_position_error):
    global integral_error
    global last_error
    integral_error += line_position_error
    derivative_error = line_position_error - last_error
    last_error = line_position_error
    return (KP * line_position_error) + (KI * integral_error) + (KD * derivative_error)

# Initialize camera sensor
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# Initialize line detection
line_detector = None

# Main loop
while(True):
    # Get a new image frame from the camera
    img = sensor.snapshot()

    # Find the left, center and right lines in the ROI
    left_line = None
    center_line = None
    right_line = None
    roi_top = int(ROI_TOP * img.height())
    roi_bottom = int(ROI_BOTTOM * img.height())
    for l in img.find_lines(thresholds=THRESHOLD_BLACK, roi=(0, roi_top, img.width(), roi_bottom), merge_distance=50):
        if l.theta() < 45 or l.theta() > 135:
            if not left_line or l.x1() < left_line.x1():
                left_line = l
            if not center_line or (l.x1() > left_line.x2() and l.x2() < right_line.x1()):
                center_line = l
            if not right_line or l.x2() > right_line.x2():
                right_line = l
    for l in img.find_lines(thresholds=THRESHOLD_WHITE, roi=(0, roi_top, img.width(), roi_bottom), merge_distance=50):
        if l.theta() < 45 or l.theta() > 135:
            if not left_line or l.x1() < left_line.x1():
                left_line = l
            if not center_line or (l.x1() > left_line.x2() and l.x2() < right_line.x1()):
                center_line = l
            if not right_line or l.x2() > right_line.x2():
                right_line = l

    # Draw a rectangle on the detected lines
    if left_line and right_line:
        img.draw_rectangle((left_line.x1(), roi_top, right_line.x2() - left_line.x1(), roi_bottom - roi_top), color=(255, 0, 0))

    # Draw a square in the region between the left and right lines
if left_line and right_line:
    x1 = left_line.x1()
    x2 = right_line.x2()
    y1 = roi_top
    y2 = roi_bottom
    x_center = (x1 + x2) // 2
    y_center = (y1 + y2) // 2
    square_size = min(x2 - x1, y2 - y1) // 2
    img.draw_rectangle((x_center - square_size, y_center - square_size, square_size * 2, square_size * 2), color=(0, 255, 0))

