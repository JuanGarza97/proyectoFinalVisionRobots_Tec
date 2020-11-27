from controller import Robot

import numpy as np
import cv2 as cv
import time

i = 0

# time in [ms] of a simulation step
TIME_STEP = 32

# create the Robot instance.
robot = Robot()

camera = robot.getCamera("CAM")
camera.enable(TIME_STEP)


def getImageAerea():
    cameraData = camera.getImageArray()
    if cameraData:
        image = np.asarray(cameraData, dtype=np.uint8)           
        #cv.imshow("preview", image)       
        camera.saveImage("fotoAerea.png",100)
        
#getImageAerea()
while robot.step(TIME_STEP) != -1:
    k = i
    pass
    #getImageAerea()
    while k < 1:
        getImageAerea()
        k += 1
    i = k
    if i <= k:
        TIME_STEP = -1

