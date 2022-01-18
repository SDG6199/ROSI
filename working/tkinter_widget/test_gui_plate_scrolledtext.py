import cv2
import tkinter 
from PIL import ImageTk, Image
from tkinter import ttk    # import tkinter 와 다른 명령이다.
from tkinter import scrolledtext
from tkinter import filedialog  # @@ tkinter.filedialog
import threading
import time
import tkinter.font as tkFont

def display_database(str):
    global flag
    while 1:
        if flag==1:
            scrolled.insert(tkinter.INSERT,str[0])
            # scrolled.insert(tkinter.END,"END text")
            scrolled.see(tkinter.END)
            flag=0
        
def get_string(arg_str):
    global flag, insert_arr,count
    while 1:
        count=count+1
        insert_arr[0]=arg_str+str(count)
        print("{}".format(insert_arr[0]))
        time.sleep(0.5)
        flag=1

def display_plate_img(img):
    global img_flag
    while 1:
        if img_flag==1:
            img1 = img[0]
            img1 = cv2.resize(img1,dsize=(0,0),fx=1.7,fy=1.7)
            img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
            img1 = Image.fromarray(img1)
            imgtk1 = ImageTk.PhotoImage(image=img1)

            lbl1.configure(image = imgtk1)

            img_flag=0

def get_plate_img():
    global img_flag, insert_img_arr
    while 1:
        insert_img_arr[0] =cv2.imread("captured car1.jpg")  
        img_flag=1
        time.sleep(0.5)
        insert_img_arr[0] =cv2.imread("test_plate.jpg")  
        img_flag=1
        time.sleep(0.5) 


if __name__ == '__main__':
    flag=0
    img_flag=0
    insert_arr=[0]   # mutable object
    insert_img_arr=[0]
    count=0

    window=tkinter.Tk()
    window.title("GUI")
    window.geometry("1920x1080+500+500")     # w,h x,y
    window.resizable(True, True)             # w,h
    C = tkinter.Canvas(window, bg="chocolate1", height=1080, width=1920)  #1. pale goldenrod 2.navajo white 3.chocolate1
    C.pack()

    fontStyle_20 = tkFont.Font(family="Lucida Grande", size=50)
    fontStyle_55 = tkFont.Font(family="Lucida Grande", size=75)
    ####

    ##  Captured Car     
    pox_x=0.08; pox_y=0.13
    ttk.Label(window, text="의심차량", font=fontStyle_20).place(relx=pox_x,rely=pox_y*0.4)

    img1 = cv2.imread("captured car1.jpg") #cv2.imread("captured car1.jpg")
    img1 = cv2.resize(img1,dsize=(0,0),fx=1.7,fy=1.7)
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
    img1 = Image.fromarray(img1)

    imgtk1 = ImageTk.PhotoImage(image=img1)
    ###img = ImageTk.PhotoImage(Image.open("captured car2.jpg"))   #image나
    ###img = tkinter.PhotoImage(file='captured car2.jpg')          #file로도 가능.
    lbl1 = tkinter.Label(window, image=imgtk1)
    ###lbl.pack(side="bottom",  anchor="e")  # bottom, east. 정확한 위치는 pack parameter를 찾아보자.  # pack 은 grid와 함께 사용될 수 없다.
    lbl1.place(relx=pox_x, rely=pox_y)


    ## Illegal parking car list
    pox_x=0.37; pox_y=0.48
    ttk.Label(window, text="단속차량 번호", font=fontStyle_20).place(relx=pox_x,rely=pox_y*0.8)
    scrolled_w = 17; scrolled_h = 4  # w, h
    scrolled = scrolledtext.ScrolledText(window, width=scrolled_w, height=scrolled_h, font=fontStyle_55,wrap=tkinter.WORD)   # 단어별로 자동개행.
    scrolled.place(relx=pox_x,rely=pox_y)

    ## Captured Plate
    pox_x=0.37; pox_y=0.13
    ttk.Label(window, text="번호판", font=fontStyle_20).place(relx=pox_x,rely=pox_y*0.4)

    img2 = cv2.imread("test_plate.jpg")
    img2 = cv2.resize(img2,dsize=(0,0),fx=1.1,fy=1.1)
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
    img2 = Image.fromarray(img2)

    imgtk2 = ImageTk.PhotoImage(image=img2)
    ###img = ImageTk.PhotoImage(Image.open("captured car2.jpg"))   #image나
    ###img = tkinter.PhotoImage(file='captured car2.jpg')          #file로도 가능.
    lbl2 = tkinter.Label(window, image=imgtk2)
    lbl2.place(x=0, y=0,relx=pox_x, rely=pox_y)

    t1 = threading.Thread(target=display_database, args=(insert_arr,))
    t1.start()
    t2 = threading.Thread(target=get_string, args=("hi",))
    t2.start()
    t3 = threading.Thread(target=display_plate_img, args=(insert_img_arr,))
    t3.start()
    t4 = threading.Thread(target=get_plate_img)
    t4.start()

    # GUI꺼질때까지 기다림
    window.mainloop()  

    t1.join()
    t2.join()