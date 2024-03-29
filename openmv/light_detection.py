# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# IR Beacon Grayscale Tracking Example
#
# This example shows off IR beacon Grayscale tracking using the OpenMV Cam.

import sensor
import time
import pyb

thresholds = (255, 255)  # thresholds for bright white light from IR.

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((240, 240))  # 240x240 center pixels of VGA
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
uart = pyb.UART(1, 9600)
while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(
                           [thresholds], pixels_threshold=200, area_threshold=200, merge=True
                       )
    if len(blobs) > 0 :
        pyb.LED(1).on()
    else:
        pyb.LED(1).off()
        uart.write('<%d,%d>' % (120, 120))

    for blob in blobs:
        ratio = blob.w() / blob.h()
        if (ratio >= 0.5) and (ratio <= 1.5):  # filter out non-squarish blobs
            uart.write('<%d,%d>' % (blob.cx(), blob.cy()))
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
    print(clock.fps())
