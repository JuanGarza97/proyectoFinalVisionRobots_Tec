# -*- coding: utf-8 -*-
"""
@author:Juan Carlos Garza Sánchez A00821522
José Andrés Miguel Martinez A01653368
Francisco Cancino Sastré A01730698
Ian Airy Suárez Barrientos A00818291
Cristian Palma Martinez A01244565
"""


import cv2
import mahotas
import numpy as np
import pickle

def getHuMoments(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    feature = cv2.HuMoments(cv2.moments(img)).flatten()
    return feature

def getHaralick(img):  # convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # compute the haralick texture feature vector
    haralick = mahotas.features.haralick(gray).mean(axis=0)
    return haralick

def getHistogram(img, mask=None):
    # convert the image to HSV color-space
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # compute the color histogram
    hist = cv2.calcHist([img], [0, 1, 2], None, [256, 256, 256], [0, 256, 0, 256, 0, 256])
    # normalize the histogram
    cv2.normalize(hist, hist)
    return hist.flatten()

def toPredict(imag, clf):

    images, contours = findObjects(imag.copy())
    
    i = 0
    prediction = []
    for img in images:
        if img is not None:
            img = cv2.resize(img, (640, 480))
            X = [(np.hstack([getHistogram(img), getHaralick(img), getHuMoments(img)]))]
            prediction = clf.predict(X)
            if len(contours) > 0:
                if prediction[0] == "circulo":
                    color = (0,0,255)
                    cv2.rectangle(imag, contours[i], color, 5)
                elif prediction[0] == "estrella":
                    color = (255, 0, 0)
                    cv2.rectangle(imag, contours[i], color, 5)
                elif prediction[0] == "triangulo":
                    color = (0, 255, 0)
                    cv2.rectangle(imag, contours[i], color, 5)
                elif prediction[0] == "rectangulo":
                    color = (0, 255, 255)
                    cv2.rectangle(imag, contours[i], color, 5)
            
        i+=1
    return prediction

def findObjects(img):
    images = []
    #img = cv2.pyrMeanShiftFiltering(img, 21, 51)
    img = cv2.GaussianBlur(img, (3, 3), 1)
    edges2 = cv2.Canny(img, 100, 200)
    kernel = np.ones((3, 3), np.uint8)
    edges2 = cv2.dilate(edges2, kernel, iterations=1)
    edges2 = cv2.erode(edges2, kernel, iterations=1)

    ret2, thresh2 = cv2.threshold(edges2, 127, 255, 0)
    contours, _ = cv2.findContours(thresh2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours2 = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            epsilon2 = 0.001 * cv2.arcLength(cnt, True)
            approx2 = cv2.approxPolyDP(cnt, epsilon2, True)
            x, y, w, h = cv2.boundingRect(approx2)
            ROI = img[y:y + h, x:x + w]
            images.append(ROI)
            r = x, y, w, h
            contours2.append(r)
    return images, contours2
    
def slidingWindow(image, clf):
    stepSize = 20
    (w_width, w_height) = (20, 20) # window size
    for x in range(0, image.shape[1] - w_width , stepSize):
        for y in range(0, image.shape[0] - w_height, stepSize):
            img = image[x:x + w_width, y:y + w_height, :]
            X = [(np.hstack([fd_hu_moments(img), fd_haralick(img), fd_hu_moments(img)]))]
            prediction = clf.predict(X)
            print (prediction)
            if prediction[0] == "circulo":
                color = (0,0,255)
                cv2.rectangle(image, (x, y), (x + w_width, y + w_height), color, 5)
            elif prediction[0] == "estrella":
                color = (255, 0, 0)
                cv2.rectangle(image, (x, y), (x + w_width, y + w_height), color, 5)
            elif prediction[0] == "triangulo":
                color = (0, 255, 0)
                cv2.rectangle(image, (x, y), (x + w_width, y + w_height), color, 5)
            elif prediction[0] == "rectangulo":
                color = (0, 255, 255)
                cv2.rectangle(image, (x, y), (x + w_width, y + w_height), color, 5)
            