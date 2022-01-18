#-*- coding: utf-8 -*-
# cpp로 darknet으로 plate 인식하려했으나 9초가 걸림.
# 파이썬으로 plate 인식하면 더 빠를 것임.

# cut_plate함수형태로 만들고 thread로 실행해야 중복호출되는 작업을 수행가능.
#std::thread _t1(cut_plate); 
#_t1.detach(); 

import os
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import shutil
import time
import json
import requests
import threading
import datetime
import gui_method

LIMIT_PX = 1024
LIMIT_BYTE = 1024*1024  # 1MB
LIMIT_BOX = 40
contour_stop_tag=0
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
        height, width, _ = height, width, _ = image.shape

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

    #image = cv2.imread(image_path)
    jpeg_image = cv2.imencode(".jpg", img)[1]
    data = jpeg_image.tobytes()

    return requests.post(API_URL, headers=headers, files={"image": data})

def OCR(img,img_out):
    
    global pkg_path, captured_car_count, contour_stop_tag

    #buffer=pkg_path+"/image/plate_cut_image/plate_cut_image"+str(captured_car_count)+".jpg"
    #image_path=buffer
    appkey = '044b56485798917360446407e2a48104'

    #resize_impath = kakao_ocr_resize(image_path)
    #if resize_impath is not None:
    #    image_path = resize_impath
    #    print("원본 대신 리사이즈된 이미지를 사용합니다.")

    output = kakao_ocr(img, appkey).json()

    #print("{}".format(json.dumps(output, sort_keys=True, indent=2,ensure_ascii=False)))
        
    if len(output["result"])==0:
        pass
    elif len(output["result"])==1:
        a=output["result"][0]["recognition_words"]
        result=list(a[0])
        if ' ' in result:
            result.remove(' ')
        print("{}".format(result))

        if len(result)==8:
            #imshow_thread_arg[0]=img_out
            result=plate_arr_to_string_sum(result)
            gui.get_string(str(result))   
            gui.get_plate_img(img_out)
            buffer=pkg_path+"/image/plate_cut_image/plate_cut_image"+str(captured_car_count)+".jpg"
            cv2.imwrite(buffer,img_out)  
            contour_stop_tag=1           # 컨투어 검사중지
    
    else:
        a=output["result"][0]["recognition_words"]
        b=output["result"][1]["recognition_words"]
        result=list(a[0])+list(b[0])
        if ' ' in result:
            result.remove(' ')
        print("{}".format(result))

        if len(result)==8:
            #imshow_thread_arg[0]=img_out
            result=plate_arr_to_string_sum(result)
            gui.get_string(str(result))
            gui.get_plate_img(img_out)
            buffer=pkg_path+"/image/plate_cut_image/plate_cut_image"+str(captured_car_count)+".jpg"
            cv2.imwrite(buffer,img_out)  
            contour_stop_tag=1          # 컨투어 검사중지

def cal_gradi(points):

    x_ls= points[:,0]
    x_ls_sorted= np.sort(x_ls)           #오름차순
    x_ls_reverse= x_ls_sorted[::-1]      #내림차순

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

    gradient=(pointa[1]-pointb[1])/(pointa[0]-pointb[0])
    return gradient

def imshow_thread(arg_img):
    global exit_flag
    cv2.namedWindow(OPENCV_WINDOW)
    cv2.resizeWindow(OPENCV_WINDOW, 370,200)
    cv2.moveWindow(OPENCV_WINDOW, 2500, 0) 
    while 1:
        cv2.imshow(OPENCV_WINDOW, arg_img[0])   
        if cv2.waitKey(1)&0xFF==27:
            exit_flag=1
            break
    return
        
#################################################################################################################################
# plate cut
def cut_plate():
    global pkg_path, captured_car_count,contour_stop_tag
    
    buffer=pkg_path+"/image/car_image/captured car"+str(captured_car_count)+".jpg"

    if os.path.exists(buffer):
        print("captured_car_count: {0} ". format(captured_car_count))
        ###############################################################################
        # 화질 개선할꺼면 화질개선 코드(shell script실행 후 프로그램 종료. canny는 그다음 실행.)#
        ###############################################################################
        
        # adaptiveThreshold로 edge detection.
        origin= cv2.imread(buffer)

        copy= origin.copy()
        copy_out=origin.copy() #출력용
        gray= cv2.cvtColor(origin, cv2.COLOR_BGR2GRAY)
        edge= cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,9,2)    
        
        # image contour, feature matching으로 rectangular detect. 
        contours, _hierachy = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
        for cnt in contours:
            epsilon= 0.01* cv2.arcLength(cnt, True)         # 둘레의 1%를 epsilon으로 할당. (낮을수록 근사정확도높음)
            approx= cv2.approxPolyDP(cnt, epsilon, True)    # 꼭지점 줄이기.
            x= approx.ravel()[0]; y= approx.ravel()[1]      # 0번째 contour index(좌표).
            _x1, _y1, w, h= cv2.boundingRect(approx)        # 외접 rect의 좌표.
            area=w*h
            if x>0 and not contour_stop_tag :  # len(approx)은 좌표의 갯수이므로 4이면 rect를 의미.
                aspectRatio= float(w)/h     # 종횡비
                if  aspectRatio >= 0 and area<4000 and area>500:     # 필요하면 aspectRatio도 조절, 너무 큰 contour도 제거 필요.@
                    print("len: {0} x:{1} y:{2} area:{3} length:{4} aspectRatio: {5}".format(len(approx),x,y,area,cv2.arcLength(cnt, True),aspectRatio))
                    rect=cv2.minAreaRect(approx)
                    pts1=cv2.boxPoints(rect)

                    #Original point(minAreaRect순서)
                    ((x_rd,y_rd),(x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru))=pts1  

                    #Change point(warpPerspective순서) 
                    gradi=cal_gradi(pts1)
                    print("{}".format(gradi))
                    if gradi<0: 
                        print("왼쪽으로 기움")
                        pts2=((x_rd,y_rd),(x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru)) 
                        #cv2.putText(copy_out,"0",(np.int0(x_rd),np.int0(y_rd)),1,2,(0,0,255),2)
                        #cv2.putText(copy_out,"1",(np.int0(x_ld),np.int0(y_ld)),1,2,(255,0,0),2)
                        #cv2.putText(copy_out,"2",(np.int0(x_lu),np.int0(y_lu)),1,2,(255,0,0),2)
                        #cv2.putText(copy_out,"3",(np.int0(x_ru),np.int0(y_ru)),1,2,(255,0,0),2)
                    else :  
                        print("오른쪽으로 기움")
                        pts2=((x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru),(x_rd,y_rd))
                        #cv2.putText(copy_out,"0",(np.int0(x_ld),np.int0(y_ld)),1,2,(0,0,255),2)
                        #cv2.putText(copy_out,"1",(np.int0(x_lu),np.int0(y_lu)),1,2,(255,0,0),2)
                        #cv2.putText(copy_out,"2",(np.int0(x_ru),np.int0(y_ru)),1,2,(255,0,0),2)
                        #cv2.putText(copy_out,"3",(np.int0(x_rd),np.int0(y_rd)),1,2,(255,0,0),2)

                    pts2=((x_rd,y_rd),(x_ld,y_ld),(x_lu,y_lu),(x_ru,y_ru)) 
                    pts2_f=np.float32(pts2)
                    pts3_f=np.float32([[0,0],[w,0],[w,h],[0,h]])           

                    mat=cv2.getPerspectiveTransform(pts2_f,pts3_f)
                    plate= cv2.warpPerspective(copy,mat,(w,h))
                    plate_out= cv2.warpPerspective(copy_out,mat,(w,h))  # 출력용

                    _,num_matched_pt=sift_matching_test(plate)
                    plate_out,_=sift_matching_test(plate_out)
                    #cv2.imwrite(buffer,plate)  # 디버깅@
                    if num_matched_pt!=0:
                         OCR(plate,plate_out)  
        captured_car_count+=1
    else:
        contour_stop_tag=0
        #print("monitoring car...")  @@
        time.sleep(1)
    
plate_arr_to_string_sum_count=[0]
now = datetime.datetime.now()

def plate_arr_to_string_sum(arg_ar):  # 앞3자리(공백제외8글자)번호판 기준. 친환경차는 7글자. @
    ret=''; ret1=''; ret2=''
    temp=arg_ar

    for i in range(0,len(temp)):
        ret=ret+temp[i]
        if not temp[i].isdigit():       #(한글이면):  
            if i==len(temp)-1:
                ret1=ret[len(temp)-4:]  
                ret2=ret[:len(temp)-4]  
            else:
                ret1=ret
                ret=''
        elif i==len(temp)-1:              #(끝나면):
            ret2=ret 

    plate_arr_to_string_sum_count[0]=plate_arr_to_string_sum_count[0]+1   
    ret=now.strftime('(%H:%M) \n')+ret1+' '+ret2  #' %Y-%m-%d %H:%M:%S'
    return ret

def sift_matching_test(arg_img):
    standard=cv2.imread('/home/sdg/Downloads/py_test/standard_plate.png',cv2.IMREAD_GRAYSCALE)
    return_img=arg_img.copy()
    compare=arg_img.copy()
    gray_compare = cv2.cvtColor(compare, cv2.COLOR_BGR2GRAY)

    sift=cv2.xfeatures2d.SIFT_create()
    kp1,des1=sift.detectAndCompute(standard,None)  # 특징점검출(detect)와 특징 디스크립터계산(compute)를 동시수행.
    kp2,des2=sift.detectAndCompute(gray_compare,None)

    bf=cv2.BFMatcher(cv2.NORM_L2,crossCheck=False)    #SIFT나 SURF는 NORM_L2, ORB나 BRIEF는 NORM_HAMMING.
    
    try: 
        matches=bf.knnMatch(des1,des2,2)
    except:
        print("Not matched.")
        return return_img, 0

        
    factor=0.85  # 첫 번째 이웃의 거리가 두 번째 이웃 거리의 85% 이내인 것만 추출. 매칭 포인트의 오차범위라고 보면 됨.
    
    if len(matches[0])!=1:   
        try:
            good_matches = [first for first,second in matches if first.distance < second.distance * factor]   
        except:
            print("{}".format(matches))

        dst_pts = np.int0([ kp2[m.trainIdx].pt for m in good_matches ])  # gray_compare 이미지의 매칭포인트의 좌표 추출.
        for i in dst_pts:
            return_img=cv2.circle(return_img,i,1,(255,255,0),1)
        return return_img, len(dst_pts)
    else:
        print("Not matched.")
        return return_img, 0


captured_car_count=1
exit_flag=0

pkg_path="/home/sdg/catkin_ws/src/image_saver"
buffer =pkg_path+"/image"
shutil.rmtree(buffer)
buffer =pkg_path+"/image/car_image"
os.makedirs(buffer)
buffer =pkg_path+"/image/plate_cut_image"
os.makedirs(buffer)

OPENCV_WINDOW="Captured Plate"

imshow_thread_arg=[0]
t1 = threading.Thread(target=imshow_thread, args=(imshow_thread_arg,))
#t1.start()

gui=gui_method.GUI()

while 1:
    cut_plate()   
    if exit_flag==1:
        break
t1.join()
cv2.destroyWindow(OPENCV_WINDOW)