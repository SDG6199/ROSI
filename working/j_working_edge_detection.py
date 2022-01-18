import cv2
import numpy as np
import matplotlib.pyplot as plt

def onChange(x):
    pass

def edge_detection2():
    img3 = cv2.imread('/home/sdg/Downloads/resolution_trash.png', cv2.IMREAD_GRAYSCALE)
    cv2.namedWindow('edge detection')
    
    cv2.createTrackbar('low threshold', 'edge detection', 0, 255, onChange)
    cv2.createTrackbar('high threshold', 'edge detection', 0, 255, onChange)
    cv2.imshow('edge detection', img3)
    
    while True:
        k = cv2.waitKey(0) & 0xFF
        
        if k == 27:
            break
            
        low = cv2.getTrackbarPos('low threshold', 'edge detection')
        high = cv2.getTrackbarPos('high threshold', 'edge detection')
        
        if low > high:
            print("Low threshold must be low than high threshold")
        
        elif ((low == 0) and (high == 0)):
            cv2.imshow('edge detection', img3)
        
        else:
            canny = cv2.Canny(img3, low, high)
            cv2.imshow('edge detection', canny)    
    cv2.destroyAllWindows()

edge_detection2()
