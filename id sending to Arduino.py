import sensor, image, time
from pyb import I2C

i2c = I2C(2, I2C.SLAVE, addr=0x42)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

while True:
    img = sensor.snapshot()
    tags = img.find_apriltags(families=image.TAG36H11)
    if len(tags) > 0:
        tag_id = tags[0].id()
        print("AprilTag detected with ID:", tag_id)
        id_bytes = [(tag_id >> 8) & 0xFF, tag_id & 0xFF]
        i2c.send(bytes(id_bytes), timeout=500)
