import numpy as np
import cv2

def find_plate():
    #Img setting 
    img= cv2.imread("/home/sdg/Downloads/resolution_good.jpg")
    output= img.copy()
    gray_img= cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    th_img= cv2.adaptiveThreshold(gray_img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,9,2)

    #Start contour
    contours, hierachy = cv2.findContours(th_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    print("{}".format(len(contours))) 
    for cnt in contours:
        approx= cv2.approxPolyDP(cnt, 0.01* cv2.arcLength(cnt, True), True) # 꼭지점 줄이기
        x= approx.ravel()[0]; y= approx.ravel()[1]      # 0번째 contour index(좌표).
        _x1, _y1, w, h= cv2.boundingRect(approx)        # 외접 rect의 좌표.
        area=w*h    #for ignore too much small boxes
        if len(approx)== 4 and x>15  :  # len(approx)은 좌표의 갯수이므로 rect를 의미.
            aspectRatio= float(w)/h     # 종횡비
            if  aspectRatio >= 2.5 and area>2500:   
                print("x:{0} y:{1} area:{2} length:{3}".format(x,y,area,cv2.arcLength(cnt, True)))
                rect=cv2.minAreaRect(approx)
                pts1=cv2.boxPoints(rect)
                pts1_i=np.int0(pts1)
                cv2.drawContours(output,[pts1_i],-1,(0,255,0),3)

                #Original point(minAreaRect순서)
                ((x_rd,y_rd),(x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru))=pts1  #시계방향 좌하부터
                print("{} {} {} {} {} {} {} {}".format(x_rd,y_rd,x_ld,y_ld,x_lu,y_lu,x_ru,y_ru))
                #Change point(warpPerspective순서) 
                pts2=((x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru),(x_rd,y_rd))  #시계방향 좌상부터
                pts2_f=np.float32(pts2)
                pts3_f=np.float32([[0,0],[w,0],[w,h],[0,h]])  #시계방향 좌상부터

                mat=cv2.getPerspectiveTransform(pts2_f,pts3_f)
                result= cv2.warpPerspective(th_img,mat,(w,h))
                #resized_img=cv2.resize(result,dsize=(750,150))
                cv2.imshow('turned_plate',result)
                
    cv2.imshow("out",output)
    cv2.waitKey(0)

find_plate()
