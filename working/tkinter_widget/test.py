import cv2
import numpy as np
import threading

def test():
    while 1:
        img1=cv2.imread('captured car1.jpg')
        print("{}".format(img1.shape))
        print("{}".format(img1))
        cv2.imshow('asd',img1)
        cv2.waitKey(1)

t1 = threading.Thread(target=test)
t1.start()

