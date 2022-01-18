# 참조: https://bkshin.tistory.com/entry/OpenCV-29-%EC%98%AC%EB%B0%94%EB%A5%B8-%EB%A7%A4%EC%B9%AD%EC%A0%90-%EC%B0%BE%EA%B8%B0?category=1148027
# feature로 안되면 Comparing histograms(색깔)을 사용해보자..

import numpy as np
import cv2

def featureMatching():
    img1=cv2.imread('/home/sdg/Downloads/py_test/standard_plate.png',cv2.IMREAD_GRAYSCALE)
    img2=cv2.imread('/home/sdg/Downloads/py_test/captured car1.png',cv2.IMREAD_GRAYSCALE)
    res=None

#---sift
    sift=cv2.xfeatures2d.SIFT_create()
    kp1,des1=sift.detectAndCompute(img1,None)  # 특징점검출(detect)와 특징 디스크립터계산(compute)를 동시수행.
    kp2,des2=sift.detectAndCompute(img2,None)
#---
#---surf(특허)
    #surf = cv2.xfeatures2d.SURF_create(1000)
    #kp1,des1=surf.detectAndCompute(img1,None)  # 특징점검출(detect)와 특징 디스크립터계산(compute)를 동시수행.
    #kp2,des2=surf.detectAndCompute(img2,None)
#---   
   
#---bf
    #bf=cv2.BFMatcher(cv2.NORM_L2,crossCheck=True)    #SIFT나 SURF는 NORM_L2, ORB나 BRIEF는 NORM_HAMMING.
    #matches=bf.match(des1,des2)
    #matches=sorted(matches,key=lambda x:x.distance)

    #res=cv2.drawMatches(img1,kp1,img2,kp2,matches[:30],res,flags=2)
#--- 
    
#---knnMatch
    bf=cv2.BFMatcher(cv2.NORM_L2,crossCheck=False)    #SIFT나 SURF는 NORM_L2, ORB나 BRIEF는 NORM_HAMMING.
    matches=bf.knnMatch(des1,des2,2)
    print("{}".format(len(matches)))
    # 첫번재 이웃의 거리가 두 번째 이웃 거리의 75% 이내인 것만 추출---⑤
    factor=0.85
    good_matches = [first for first,second in matches \
                    if first.distance < second.distance * factor]   # list comprehension
    print('good_matches :%d/%d' %(len(good_matches),len(matches)))

    # 좋은 매칭점의 queryIdx로 원본 영상의 좌표 구하기 ---③
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches ])
    # 좋은 매칭점의 trainIdx로 대상 영상의 좌표 구하기 ---④
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ])
    src_pts_i=np.int0(src_pts)
    dst_pts_i=np.int0(dst_pts)

    # 원근 변환 행렬 구하기 ---⑤
    mtrx, mask = cv2.findHomography(src_pts, dst_pts)
    # 원본 영상 크기로 변환 영역 좌표 생성 ---⑥
    h,w, = img1.shape[:2]
    pts = np.float32([ [[0,0]],[[0,h-1]],[[w-1,h-1]],[[w-1,0]] ])
    # 원본 영상 좌표를 원근 변환  ---⑦
    dst = cv2.perspectiveTransform(pts,mtrx)
    # 변환 좌표 영역을 대상 영상에 그리기 ---⑧
    img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

    # 좋은 매칭만 그리기
    res = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, res, \
                        flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

#---대상 영상의 매칭점 좌표얻기 

    #print("{}".format(dst_pts_i))
    for i in dst_pts_i:
        img2=cv2.circle(img2,i,5,(0,255,0),2)
#---   
#---FLANN
    #FLANN_INDEX_KDTREE=0
    #index_params=dict(algorithm=FLANN_INDEX_KDTREE,trees=5)
    #search_params=dict(checks=50)

    #flann=cv2.FlannBasedMatcher(index_params,search_params)
    #matches=flann.knnMatch(des1,des2,k=2)
    #factor=0.7
    #good=[]
    #for m,n in matches:
    #    if m.distance < factor*n.distance:
    #        good.append(m)
    
    #res=cv2.drawMatches(img1,kp1,img2,kp2,good,res,flags=2)
#---
    #res=cv2.drawKeypoints(img2,kp2,res,(0,0,255),flags=0)  
    cv2.imshow('Feature Matching', res)
    cv2.waitKey(0)
    cv2.imshow('second Matching', img2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

featureMatching()