import sensor, image, time, math
import pyb,ustruct


# Initialize I2C communication as slave
# Note: The I2C hardware bus on OpenMV Cam is bus 2.
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
bus.deinit()  # Completely shut down the device
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
print("Waiting for Arduino...")

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
    intersection = False

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
    lines = img.find_lines(roi=(0, int(ROI_TOP*img.height()), img.width(), int(ROI_BOTTOM*img.height())), merge_distance = 30)
    # If we detect more than 3 lines, we consider it an intersection
    if len(lines) > 4:
        print("Intersection detected!")
        intersection = False
    # Detect center line as midpoint of left and right line
    if left_line and right_line:
        center_x = (left_line.x1() + right_line.x1()) // 2
        center_y = (left_line.y1() + right_line.y1()) // 2
        center_line = (center_x, center_y)

    # PID control
    if center_line:
        line_position_error = img.width()//2 - center_line[0]
        pid_output = pid_control(line_position_error)
        servo_position = 90 - pid_output
        print("servo_position : ", servo_position)
        servo_position=str(servo_position)
        servo_position = ustruct.pack("<%ds" % len(servo_position), servo_position)

    # Draw lines
    for l in lines:
        if intersection:
            img.draw_line(l.line(), color=(255,255,0)) # Highlight lines that form the intersection
        else:
            img.draw_line(l.line(), color=(0,0,255)) # Normal line color when no intersection

    if center_line: img.draw_cross(center_line[0], center_line[1], color=(0,255,0)) # Green cross for center line

    # Send output via I2C
    if intersection:
        crosswalk ="intersection"
        crosswalk = ustruct.pack("<%ds" % len(crosswalk), crosswalk)
        try:
            bus.send(ustruct.pack("<h", len(crosswalk)), timeout=10000)  # Send the length (16-bits) first
            try:
                bus.send(crosswalk, timeout=10000)  # Then send the data
                print("Sent Data!")  # Displayed when no errors occur
            except OSError as err:
                pass  # Skip if an error occurs
        except OSError as err:
            pass  # Skip if an error occurs
    if not intersection:  # Do not send servo position if we're at an intersection
        try:
            bus.send(ustruct.pack("<h", len(servo_position)), timeout=10000)  # Send the length (16-bits) first
            try:
                bus.send(servo_position, timeout=10000)  # Then send the data
                print("Sent Data!")  # Displayed when no errors occur
            except OSError as err:
                pass  # Skip if an error occurs
        except OSError as err:
            pass  # Skip if an error occurs
