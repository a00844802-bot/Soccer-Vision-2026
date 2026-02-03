import sensor
import time
import utime
import pyb
import math
import image
from pyb import UART

# ================= RESOLUCIÓN =================
ANCHO = 320
ALTO = 240

# ================= PARÁMETROS =================
AREA_MINIMA_CERCA = 100
AREA_MINIMA_LEJOS = 4
PIXELS_MIN = 3

# ================= UART =================
uart = UART(3, 115200, timeout_char=0)
uart.init(115200, bits=8, parity=None, stop=1)

# ================= THRESHOLDS =================
TH_BALL   = (40, 100, 30, 127, 20, 127)
TH_GOAL_Y = (35, 100, -128, -10, -128, -10) #(50, 73, -55, 48, 8, 28)
TH_GOAL_B = (35, 100, -128, -10, -128, -10)

# ================= CENTRO =================
X_CENTER = ANCHO // 2
Y_CENTER = ALTO // 2

# ================= INIT =================
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

# ================= UTIL =================
def get_biggest_blob(blobs):
    return max(blobs, key=lambda b: b.area()) if blobs else None

def distance(cx, cy):
    return math.sqrt((cx - X_CENTER) ** 2 + (cy - Y_CENTER) ** 2)

def angle(cx, cy):
    a = math.degrees(math.atan2(cy - Y_CENTER, cx - X_CENTER))
    return a + 360 if a < 0 else a

# ================= MAIN =================
def main():
    initialize_open()
    clock = time.clock()

    while True:
        clock.tick()
        img = sensor.snapshot()

        # ================= PELOTA =================
        distance_b = 0
        angle_b = 0

        blobs = img.find_blobs(
            [TH_BALL],
            pixels_threshold=PIXELS_MIN,
            area_threshold=AREA_MINIMA_LEJOS,
            merge=True
        )

        if blobs:
            # prioriza blobs grandes y cercanos al centro
            blobs = sorted(
                blobs,
                key=lambda b: (b.area(), -distance(b.cx(), b.cy())),
                reverse=True
            )

            for blob in blobs:
                area = blob.area()
                circularidad = blob.compactness()

                # pelota cerca 
                if area >= AREA_MINIMA_CERCA and circularidad > 0.3:
                    valido = True

                # pelota lejos (Casi un punto/cuidado cables) 
                elif AREA_MINIMA_LEJOS <= area < AREA_MINIMA_CERCA:
                    valido = True

                else:
                    valido = False

                if valido:
                    img.draw_rectangle(blob.rect(), color=(255, 255, 255))
                    img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 255))

                    if area >= AREA_MINIMA_CERCA:
                        radio = int((blob.w() + blob.h()) / 4)
                        img.draw_circle(blob.cx(), blob.cy(), radio, color=(255,255,255))

                    distance_b = distance(blob.cx(), blob.cy())
                    angle_b = -(angle(blob.cx(), blob.cy()) - 180)
                    break

        # ================= PORTERÍA AMARILLA =================
        distance_g = 0
        angle_g = 0

        blobs = img.find_blobs(
            [TH_GOAL_Y],
            pixels_threshold=200,
            area_threshold=800,
            merge=True
        )

        blob = get_biggest_blob(blobs)
        if blob:
            img.draw_rectangle(blob.rect(), color=(0, 255, 0))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
            distance_g = distance(blob.cx(), blob.cy())
            angle_g = -(angle(blob.cx(), blob.cy()) - 180)

        # ================= PORTERÍA AZUL =================
        distance_gop = 0
        angle_gop = 0

        blobs = img.find_blobs(
            [TH_GOAL_B],
            pixels_threshold=200,
            area_threshold=800,
            merge=True
        )

        blob = get_biggest_blob(blobs)
        if blob:
            img.draw_rectangle(blob.rect(), color=(0, 0, 255))
            img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 255))
            distance_gop = distance(blob.cx(), blob.cy())
            angle_gop = -(angle(blob.cx(), blob.cy()) - 180)

        # ================= UART =================
        data = "{} {} {} {} {} {}\n".format(
            distance_b, angle_b,
            distance_g, angle_g,
            distance_gop, angle_gop
        )

        print("Sending:", data)
        uart.write(data)
        pyb.delay(50)

# ================= START =================
main()