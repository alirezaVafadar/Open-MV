import sensor, image, time, math, pyb, ustruct

GRAYSCALE_THRESHOLD = (255, 255)
BINARY_VIEW = True

MAG_THRESHOLD = 4
THETA_GAIN = 40.0
RHO_GAIN = -1.0
P_GAIN = 0.7
I_GAIN = 0.0
I_MIN = -0.0
I_MAX = 0.0
D_GAIN = 0.1

# Initialize the I2C communication
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
clock = time.clock()

THRESHOLD_BLACK = (0, 50)
THRESHOLD_WHITE = (200, 255)


def line_to_theta_and_rho(line):
    if line.rho() < 0:
        if line.theta() < 90:
            return (math.sin(math.radians(line.theta())),
                math.cos(math.radians(line.theta() + 180)) * -line.rho())
        else:
            return (math.sin(math.radians(line.theta() - 180)),
                math.cos(math.radians(line.theta() + 180)) * -line.rho())
    else:
        if line.theta() < 90:
            if line.theta() < 45:
                return (math.sin(math.radians(180 - line.theta())),
                    math.cos(math.radians(line.theta())) * line.rho())
            else:
                return (math.sin(math.radians(line.theta() - 180)),
                    math.cos(math.radians(line.theta())) * line.rho())
        else:
            return (math.sin(math.radians(180 - line.theta())),
                math.cos(math.radians(line.theta())) * line.rho())

def line_to_theta_and_rho_error(line, img):
    t, r = line_to_theta_and_rho(line)
    return (t, r - (img.width() // 2))

old_result = 0
old_time = pyb.millis()
i_output = 0
output = 90

while(True):
    clock.tick()
    img = sensor.snapshot()

    if BINARY_VIEW:
        img.binary([GRAYSCALE_THRESHOLD])
        img.erode(1)

    line = img.get_regression([(255,255) if BINARY_VIEW else THRESHOLD], robust = True)

    if line and (line.magnitude() >= MAG_THRESHOLD):
        img.draw_line(line.line(), color=127)

        t, r = line_to_theta_and_rho_error(line, img)
        new_result = (t * THETA_GAIN) + (r * RHO_GAIN)
        delta_result = new_result - old_result
        old_result = new_result

        new_time = pyb.millis()
        delta_time = new_time - old_time
        old_time = new_time

        p_output = new_result
        i_output = max(min(i_output + new_result, I_MAX), I_MIN)
        d_output = (delta_result * 1000) / delta_time
        pid_output = (P_GAIN * p_output) + (I_GAIN * i_output) + (D_GAIN * d_output)

        output = 90 + max(min(int(pid_output), 90), -90)
        print_string = "Line Ok - turn %d - line t: %d, r: %d" % (output, line.theta(), line.rho())
    else:
        print_string = "Line Lost - turn %d" % output

    servo_position = ((180 - output) / 180) * 60 + 60
    print("Servo position: ", servo_position)

    servo_position = str(servo_position)
    servo_position = ustruct.pack("<%ds" % len(servo_position), servo_position)

    # Send the servo position via I2C
    try:
        bus.send(ustruct.pack("<h", len(servo_position)), timeout=10000)
        try:
            bus.send(servo_position, timeout=10000)
            print("Sent Data!")
        except OSError as err:
            pass
    except OSError as err:
        pass

    print("FPS %f, mag = %s" % (clock.fps(), str(line.magnitude()) if line else "N/A"))
    img.invert()
    time.sleep(0.1)
