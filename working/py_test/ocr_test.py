import numpy as np
import cv2

def featureMatching():
    img1=cv2.imread('/home/sdg/Downloads/py_test/test2_num1.png',cv2.IMREAD_GRAYSCALE)
    img2=cv2.imread('/home/sdg/Downloads/py_test/captured car1.jpg',cv2.IMREAD_GRAYSCALE)
    res=None

    orb=cv2.ORB_create()
    kp1,des1=orb.detectAndCompute(img1,None)
    kp2,des2=orb.detectAndCompute(img2,None)
    
    bf=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
    matches=bf.match(des1,des2)
    matches=sorted(matches,key=lambda x:x.distance)
    
    res=cv2.drawMatches(img1,kp1,img2,kp2,matches[:30],res,flags=2)

    cv2.imshow('Feature Matching', res)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

featureMatching()

