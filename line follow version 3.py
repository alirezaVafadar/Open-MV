import sensor, image, time, math, pyb, ustruct

bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
bus.deinit()
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
print("Waiting for Arduino...")

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

THRESHOLD = [(200, 255)] # grayscale threshold for robust regression

ROI_TOP = 0.5
ROI_BOTTOM = 1.0

KP = 0.5
KI = 0
KD = 0

SOLID_LINE_ERROR = 0
DASHED_LINE_ERROR = 0
INTEGRAL_ERROR_SOLID = 0
INTEGRAL_ERROR_DASHED = 0
LAST_ERROR_SOLID = 0
LAST_ERROR_DASHED = 0
PID_OUTPUT = 0

def pid_control(line_position_error, last_error, integral_error):
    integral_error += line_position_error
    derivative_error = line_position_error - last_error
    last_error = line_position_error
    return (KP * line_position_error) + (KI * integral_error) + (KD * derivative_error), last_error, integral_error

while(True):
    img = sensor.snapshot().histeq()  # histogram equalization enhances contrast
    img.invert()

    lines = img.find_lines()
    solid_line = None
    dashed_line = None

    # Distinguish the dashed line and the solid line
    for l in lines:
        if l.y2() > img.height() * 3 // 4:  # Closer to the bottom is solid
            if solid_line is None or l.magnitude() > solid_line.magnitude():
                solid_line = l
        else:  # Closer to the top is dashed
            if dashed_line is None or l.magnitude() > dashed_line.magnitude():
                dashed_line = l

    if solid_line is not None:
        SOLID_LINE_ERROR, LAST_ERROR_SOLID, INTEGRAL_ERROR_SOLID = pid_control(
            img.width()//2 - solid_line.x1(), LAST_ERROR_SOLID, INTEGRAL_ERROR_SOLID)
    if dashed_line is not None:
        DASHED_LINE_ERROR, LAST_ERROR_DASHED, INTEGRAL_ERROR_DASHED = pid_control(
            img.width()//2 - dashed_line.x1(), LAST_ERROR_DASHED, INTEGRAL_ERROR_DASHED)

    if dashed_line is not None and solid_line is not None:
        PID_OUTPUT = (SOLID_LINE_ERROR + DASHED_LINE_ERROR) / 6
    elif dashed_line is not None:
        PID_OUTPUT = DASHED_LINE_ERROR
    else:
        PID_OUTPUT = SOLID_LINE_ERROR

    if solid_line is not None: img.draw_line(solid_line.line(), color=255)  # draw the solid line
    if dashed_line is not None: img.draw_line(dashed_line.line(), color=255)  # draw the dashed line

    servo_position = 90 - int(PID_OUTPUT)
    print("servo_position : ", servo_position)
    servo_position = str(servo_position)
    servo_position = ustruct.pack("<%ds" % len(servo_position), servo_position)

    try:
        bus.send(ustruct.pack("<h", len(servo_position)), timeout=10000)
        try:
            bus.send(servo_position, timeout=10000)
            print("Sent Data!")
        except OSError as err:
            pass
    except OSError as err:
        pass
