
import cv2


name="num"
path='/home/jelly/catkin_ws/src/Yolo_mark/x64/Release/data/img/'
count=1
last=2814
while count<=last : 
    try:
        txt = open(path+name+str(count)+'.txt','r')
        line = txt.readline() 
        if not line: 
            print(count)
            img= cv2.imread(path+name+str(count)+'.jpg')
            cv2.imshow(path+name+str(count)+'.jpg',img)
            cv2.waitKey(500)
            
    except:
        pass


        

    count=count+1    
txt.close() 
