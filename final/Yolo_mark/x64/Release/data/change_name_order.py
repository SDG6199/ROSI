
import cv2


name="num"
path='/home/jelly/catkin_ws/src/Yolo_mark/x64/Release/data/'
count=1954
last=1954
while count<=last : 
    try:
        txt = open(path+name+str(count)+'.txt','r+')
        print(count)
        lines = txt.readlines()
        print(lines)
        for line in lines:
            if line[0]=="0": 
                print("this is 0 line") 
                lines[0].replace('0','8',1)
            if line[0]=="1":
                print("this is 1 line")
                lines[0].replace('1','9',1)
        print(lines)
        #txt = open(path+name+str(count)+'.txt','w')
        #txt.write(lines)

        if not line: 
            print(count)
            img= cv2.imread(path+name+str(count)+'.jpg')
            cv2.imshow(path+name+str(count)+'.jpg',img)
            cv2.waitKey(500)
            continue
        
    except:
        pass


        

    count=count+1    
txt.close() 
