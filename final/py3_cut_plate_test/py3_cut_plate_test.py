import os
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import shutil
import time
import json
import requests
LIMIT_PX = 1024
LIMIT_BYTE = 1024*1024  # 1MB
LIMIT_BOX = 40

#################################################################################################################################
# OCR (kakao API)
def kakao_ocr_resize(image_path: str):
    """
    ocr detect/recognize api helper
    ocr api의 제약사항이 넘어서는 이미지는 요청 이전에 전처리가 필요.

    pixel 제약사항 초과: resize
    용량 제약사항 초과  : 다른 포맷으로 압축, 이미지 분할 등의 처리 필요. (예제에서 제공하지 않음)

    :param image_path: 이미지파일 경로
    :return:
    """
    image = cv2.imread(image_path)
    height, width, _ = image.shape

    if LIMIT_PX < height or LIMIT_PX < width:
        ratio = float(LIMIT_PX) / max(height, width)
        image = cv2.resize(image, None, fx=ratio, fy=ratio)
        height, width, _ = image.shape

        # api 사용전에 이미지가 resize된 경우, recognize시 resize된 결과를 사용해야함.
        image_path = "{}_resized.jpg".format(image_path)
        cv2.imwrite(image_path, image)

        return image_path
    return None

def kakao_ocr(img, appkey: str):
    """
    OCR api request example
    :param image_path: 이미지파일 경로
    :param appkey: 카카오 앱 REST API 키 : 
    """
    API_URL = 'https://dapi.kakao.com/v2/vision/text/ocr'

    headers = {'Authorization': 'KakaoAK {}'.format(appkey)}

    jpeg_image = cv2.imencode(".jpg", img)[1]
    data = jpeg_image.tobytes()

    return requests.post(API_URL, headers=headers, files={"image": data})

def OCR(img):
    global pkg_path, captured_car_count
  
    appkey = '044b56485798917360446407e2a48104'
    output = kakao_ocr(img, appkey).json()

    if len(output["result"])==0:
        pass
    elif len(output["result"])==1:
        print("{}".format(json.dumps(output, sort_keys=True, indent=2, ensure_ascii=False)))
        print("[OCR]:\n{}\n".format(output["result"][0]["recognition_words"]))
        a=output["result"][0]["recognition_words"]
        result=[list(a[0])]
        result=sum(result,[])
        if ' ' in result:
            result.remove(' ')
        print("{}".format(result))
        if len(result)==8:
            pass
    else: 
        print("{}".format(json.dumps(output, sort_keys=True, indent=2,ensure_ascii=False)))
        print("[OCR]:\n{}\n".format(output["result"][0]["recognition_words"]))
        a=output["result"][0]["recognition_words"]
        print("[OCR]:\n{}\n".format(output["result"][1]["recognition_words"]))
        b=output["result"][1]["recognition_words"]

        result=[list(a[0])+list(b[0])]
        result=sum(result,[])
        if ' ' in result:
            result.remove(' ')
        print("{}".format(result))
        if len(result)==8:
            pass

def cal_gradi(points):
    #((x_0,y_0),(x_1,y_1),(x_2,y_2),(x_3,y_3))=points 
    x_ls= points[:,0]                  
    x_ls_sorted= np.sort(x_ls)              #오름차순
    x_ls_reverse= x_ls_sorted[::-1]         #내림차순

    if points[np.where(x_ls<x_ls_sorted[2])][0][1] > points[np.where(x_ls<x_ls_sorted[2])][1][1]:
    #if x작은순서로 상위2개인 포인트1의 y > x작은순서로 상위2개인 포인트2의 y:
        pointa=points[np.where(x_ls<x_ls_sorted[2])][1]
    else:
        pointa=points[np.where(x_ls<x_ls_sorted[2])][0]

    if points[np.where(x_ls>x_ls_reverse[2])][0][1] > points[np.where(x_ls>x_ls_reverse[2])][1][1]:  
    #if x큰순서로 상위2개인 포인트1의 y > x큰순서로 상위2개인 포인트2의 y:
        pointb= points[np.where(x_ls>x_ls_reverse[2])][1]
    else:
        pointb= points[np.where(x_ls>x_ls_reverse[2])][0]

    print("point_a: {} point_b: {}".format(pointa,pointb))
    gradient=(pointa[1]-pointb[1])/(pointa[0]-pointb[0])
    return gradient

def find_plate():
    #Img setting 
    captured_car_count=14   #@
    
    pkg_path="/home/sdg/Downloads/py3_cut_plate_test"
    buffer=pkg_path+"/captured car"+str(captured_car_count)+".jpg"

    origin= cv2.imread(buffer)
    copy_p= origin.copy()
    copy_b= origin.copy()
    gray= cv2.cvtColor(origin, cv2.COLOR_BGR2GRAY)
    edge= cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,9,2)    

##
    # image contour로 rectangular detect.
    contours, _hierachy = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    for cnt in contours:
        epsilon= 0.01* cv2.arcLength(cnt, True)         # 둘레의 1%를 epsilon으로 할당. (낮을수록 근사정확도높음)
        approx= cv2.approxPolyDP(cnt, epsilon, True)    # 꼭지점 줄이기.
        x= approx.ravel()[0]; y= approx.ravel()[1]      # 0번째 contour 좌표저장. ravel:Return a contiguous flattened array.
        _x1, _y1, w, h= cv2.boundingRect(approx)        # 외접 rect의 좌표.
        area=w*h
        if x>0  :  # len(approx)은 좌표의 갯수이므로 rect를 의미. len(approx)== 4 and
            aspectRatio= float(w)/h     # 종횡비
            if  aspectRatio >= 0 and area>100:   
                print("len: {0} x:{1} y:{2} area:{3} length:{4} aspectRatio: {5}".format(len(approx),x,y,area,cv2.arcLength(cnt, True),aspectRatio))
                rect=cv2.minAreaRect(approx)
                pts1=cv2.boxPoints(rect)

                #Original point(minAreaRect순서)  
                ((x_rd,y_rd),(x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru))=pts1  
                #print("{}".format(pts1))
                
                #Change point(warpPerspective순서) 
                gradi=cal_gradi(pts1)
                print("{}".format(gradi))
                if gradi<0: 
                    print("왼쪽으로 기움")
                    pts2=((x_rd,y_rd),(x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru)) 
                    cv2.putText(copy_b,"0",(np.int0(x_rd),np.int0(y_rd)),1,2,(0,0,255),2)
                    cv2.putText(copy_b,"1",(np.int0(x_ld),np.int0(y_ld)),1,2,(255,0,0),2)
                    cv2.putText(copy_b,"2",(np.int0(x_lu),np.int0(y_lu)),1,2,(255,0,0),2)
                    cv2.putText(copy_b,"3",(np.int0(x_ru),np.int0(y_ru)),1,2,(255,0,0),2)
                else :  
                    print("오른쪽으로 기움")
                    pts2=((x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru),(x_rd,y_rd))
                    cv2.putText(copy_b,"0",(np.int0(x_ld),np.int0(y_ld)),1,2,(0,0,255),2)
                    cv2.putText(copy_b,"1",(np.int0(x_lu),np.int0(y_lu)),1,2,(255,0,0),2)
                    cv2.putText(copy_b,"2",(np.int0(x_ru),np.int0(y_ru)),1,2,(255,0,0),2)
                    cv2.putText(copy_b,"3",(np.int0(x_rd),np.int0(y_rd)),1,2,(255,0,0),2)

                pts2_f=np.float32(pts2)   
                pts3_f=np.float32([[0,0],[w,0],[w,h],[0,h]])           
                
                mat=cv2.getPerspectiveTransform(pts2_f,pts3_f)
                plate= cv2.warpPerspective(copy_p,mat,(w,h))
                #plate=cv2.resize(plate,dsize=(1024,1024))
                #resized_img=cv2.resize(result,dsize=(750,150))
                buffer=pkg_path+"/plate_cut_image"+str(captured_car_count)+".jpg"
                cv2.imwrite(buffer,plate)
                #cv2.imshow('captured plate',plate)
                #cv2.waitKey(0)
                #plate box
                pts1_i=np.int0(pts1)
                cv2.drawContours(copy_b,[pts1_i],0,(0,255,0),3)
                buffer=pkg_path+"/plate_box"+str(captured_car_count)+".jpg"
                cv2.imwrite(buffer,copy_b)

                OCR(plate)

    buffer=pkg_path+"/edge"+str(captured_car_count)+".jpg"
    cv2.imwrite(buffer,edge)  #edge
find_plate()
