'''
 * file Vision-Frontal.py
 * author Jared Aldana Palacios 
 * brief Source file for the OpenMV
 * date 2026-02-12
'''

import sensor
import time
import utime
import pyb
import math
import image
from pyb import UART

# IMAGE RESOLUTION
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# ADJUSTABLE PARAMETERS
MIN_CLOSE_AREA = 100
MIN_FAR_AREA = 4
MON_PIXELS = 3

# UART COMMUNICATION 
uart = UART(3, 115200, timeout_char=0)
uart.init(115200, bits=8, parity=None, stop=1)

# THRESHOLDS
BALL_COLOR_THRESHOLD   = (40, 100, 30, 127, 20, 127)
THRESHOLD_YELLOW_GOAL = (50, 73, -55, 48, 8, 28)
THRESHOLD_BLUE_GOAL = (35, 100, -128, -10, -128, -10)

# IMAGE CENTER 
X_CENTER = CAMERA_WIDTH // 2
Y_CENTER = CAMERA_HEIGHT // 2

# INITIALIZATION 
def initialize_open():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=3000)

    sensor.set_auto_gain(False)
    sensor.set_gainceiling(16)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=45000)

    sensor.set_brightness(-1)
    sensor.set_contrast(-4)
    sensor.set_saturation(-6)

    sensor.set_hmirror(True)
    sensor.set_transpose(True)


def get_biggest_blob(blobs):
    return max(blobs, key=lambda b: b.area()) if blobs else None

def distance(cx, cy):
    return math.sqrt((cx - X_CENTER) ** 2 + (cy - Y_CENTER) ** 2)

def angle(cx, cy):
    a = math.degrees(math.atan2(cy - Y_CENTER, cx - X_CENTER))
    return a + 360 if a < 0 else a

# MAIN LOOP
def main():
    initialize_open()
    clock = time.clock()

    while True:
        clock.tick()
        img = sensor.snapshot()

        # BALL DETECTION
        distance_ball = 0
        angle_ball = 0

        blobs = img.find_blobs(
            [BALL_COLOR_THRESHOLD],
            pixels_threshold=MON_PIXELS,
            area_threshold=MIN_FAR_AREA,
            merge=True
        )

        if blobs:
             # Prioritize larger blobs and those closer to the image center
            blobs = sorted(
                blobs,
                key=lambda IMAGEBLOB: (IMAGEBLOB.area(), -distance(IMAGEBLOB.cx(), IMAGEBLOB.cy())),
                reverse=True
            )

            for blob in blobs:
                area = blob.area()
                circularity = blob.roundness() # roundness = (4π × área) / (perímeter²) Values between 0 and 1, being 1 a perfect circle
                #circularity= blob.compactness() # compactness = (perímeter²) / (4π × área) Values between 1 and ∞, being 1 a perfect circle

                # Near ball (Larger and more circular blobs)
                if area >= MIN_CLOSE_AREA and circularity > 0.7:
                    valid_blob = True

                # Far ball (Almost a point/caution cables) 
                elif MIN_FAR_AREA <= area < MIN_CLOSE_AREA:
                    valid_blob = True

                else:
                    valid_blob = False

                if valid_blob:
                    img.draw_rectangle(blob.rect(), color=(255, 255, 255))
                    img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 255))

                    if area >= MIN_CLOSE_AREA:
                        radio = int((blob.w() + blob.h()) / 4)
                        img.draw_circle(blob.cx(), blob.cy(), radio, color=(255,255,255))

                    distance_ball = distance(blob.cx(), blob.cy())
                    angle_ball = -(angle(blob.cx(), blob.cy()) - 180)
                    break

        # YELLOW GOAL DETECTION
        distance_yellow_goal = 0
        angle_yellow_goal = 0

        blobs = img.find_blobs(
            [THRESHOLD_YELLOW_GOAL],
            pixels_threshold=200,
            area_threshold=800,
            merge=True
        )

        blob = get_biggest_blob(blobs)
        if blob:
            img.draw_rectangle(blob.rect(), color=(0, 255, 0))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
            distance_yellow_goal = distance(blob.cx(), blob.cy())
            angle_yellow_goal = -(angle(blob.cx(), blob.cy()) - 180)

        # BLUE GOAL DETECTION
        distance_blue_goal = 0
        angle_blue_goal = 0

        blobs = img.find_blobs(
            [THRESHOLD_BLUE_GOAL],
            pixels_threshold=200,
            area_threshold=800,
            merge=True
        )

        blob = get_biggest_blob(blobs)
        if blob:
            img.draw_rectangle(blob.rect(), color=(0, 0, 255))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))
            distance_blue_goal = distance(blob.cx(), blob.cy())
            angle_blue_goal = -(angle(blob.cx(), blob.cy()) - 180)

        # SEND DATA VIA UART
        data = "{} {} {} {} {} {}\n".format(
            distance_ball, angle_ball,
            distance_yellow_goal, angle_yellow_goal,
            distance_blue_goal, angle_blue_goal
        )

        print("Sending:", data)
        uart.write(data)
        pyb.delay(50)

# START
main()