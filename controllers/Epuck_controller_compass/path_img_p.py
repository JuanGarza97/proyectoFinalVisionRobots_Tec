# -*- coding: utf-8 -*-
"""

@author: José Andrés Miguel Martinez A01653368
Juan Carlos Garza Sánchez A00821522
Francisco Cancino Sastré A01730698
Ian Airy Suárez Barrientos A00818291
Cristian Palma Martinez A01244565

"""

import cv2
import numpy as np

kernel_size = 3
img = None
while img is None:
    img = cv2.imread('../air_Cam_Controller/fotoAerea.png')
img = cv2.resize(img, (450, 450))

#img = cv2.resize(img,(450,450),interpolation=cv2.INTER_AREA)
img_canny = cv2.GaussianBlur(img, (3,3), 0)
img_canny = cv2.Canny(img_canny, 60, 127, apertureSize=3)


kernel = np.ones((20, 20), np.uint8)
img_d = cv2.dilate(img_canny, kernel, iterations=1)

# img_d = img_canny

cv2.imshow('d', img_d)
cv2.waitKey(30)
seg = 10



def segmentar(kx_size, ky_size):
    global img_d
    y, x = img_d.shape
    mem = np.zeros((y // kx_size, x // ky_size), dtype=np.uint8)
    x_mem = y_mem = 0
    for i in range(0, y, ky_size):
        x_mem = 0
        for j in range(0, x, kx_size):
            mem[y_mem, x_mem] = np.max(img_d[i:i + ky_size,
                                       j:j + kx_size])
            x_mem += 1
        y_mem += 1
    return mem


mem = segmentar(seg, seg)
mem_y, mem_x = mem.shape
mem2 = cv2.resize(mem, (450, 450), interpolation=cv2.INTER_AREA)

print(mem.shape)
pard_list = []


def paredes():
    global mem
    global pard_list
    y, x = mem.shape
    for i in range(0, y):
        for j in range(0, x):
            if mem[i, j] == 255:
                pard_list.append((i, j))


cv2.imshow('segmt0', mem2)
cv2.waitKey(30)
paredes()
cv2.imwrite('Photos/El_org.png', mem2)


