import cv2
import tkinter 
from PIL import ImageTk, Image
from tkinter import ttk    # import tkinter 와 다른 명령이다.
from tkinter import scrolledtext
import threading
import time
import tkinter.font as tkFont
import numpy as np
import os
# scrolledtext를 안쓰고 scroll과 text widget만으로도 구현가능하다. https://stackoverflow.com/questions/30669015/autoscroll-of-text-and-scrollbar-in-python-text-box
# from tkinter import filedialog는 tkinter에 사용할 데이터를 txt파일로부터 load하는 기능.

class GUI():
    def __init__(self):
        print("thread_display_database start.")

        self.insert_flag=[0]
        self.img_flag=[0]
        self.insert_arr=["#Illegal parking car list"]   # mutable object
        self.insert_img_arr=[0]
        self.t1 = threading.Thread(target=self.display_window)   
        self.t1.start()

    def display_window(self):
        window=tkinter.Tk()
        window.title("GUI")
        window.geometry("960x1080+960+0")     # w,h x,y
        window.resizable(True, True)             # w,h
        C = tkinter.Canvas(window, bg="chocolate1", height=1080, width=1920)  #1. pale goldenrod 2.navajo white 3.chocolate1
        C.pack()
        self.fontStyle_title = tkFont.Font(family="Lucida Grande", size=50)  #Helvetica
        self.fontStyle_text = tkFont.Font(family="Lucida Grande", size=35)
        ####
        ##  Captured Car     
        pox_x=0.04; pox_y=0.17
        ttk.Label(window, text="의심차량", font=self.fontStyle_title).place(relx=pox_x,rely=pox_y*0.4)

        img1 = cv2.imread("/home/sdg/catkin_ws/src/image_saver/script/waitng_car.png")
        img1 = cv2.resize(img1, (350, 350), interpolation = cv2.INTER_CUBIC)
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
        img1 = Image.fromarray(img1)

        imgtk1 = ImageTk.PhotoImage(image=img1)
        self.lbl1 = tkinter.Label(window, image=imgtk1) # 여러 개 띄우고 싶으면, master=[window명]으로 인자를 넣어주면 됨.
        self.lbl1.place(relx=pox_x, rely=pox_y)
       
        ## Illegal parking car list
        pox_x=0.47; pox_y=0.48
        ttk.Label(window, text="단속차량 번호", font=self.fontStyle_title).place(relx=pox_x,rely=pox_y*0.8)

        scrolled_w = 14; scrolled_h = 9  # w, h
        self.scrolled = scrolledtext.ScrolledText(window, width=scrolled_w, height=scrolled_h, font=self.fontStyle_text,wrap=tkinter.WORD)   # 단어별로 자동개행.
        self.scrolled.place(relx=pox_x,rely=pox_y)

        ## Captured Plate
        pox_x=0.47; pox_y=0.17
        ttk.Label(window, text="번호판", font=self.fontStyle_title).place(relx=pox_x,rely=pox_y*0.4)

        img2 = cv2.imread("/home/sdg/catkin_ws/src/image_saver/script/waiting_plate.jpg")
        img2 = cv2.resize(img2, (400, 150), interpolation = cv2.INTER_CUBIC)
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
        img2 = cv2.resize(img2,dsize=(0,0),fx=1.1,fy=1.1)
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
        img2 = Image.fromarray(img2)

        imgtk2 = ImageTk.PhotoImage(image=img2)
        self.lbl2 = tkinter.Label(window, image=imgtk2)
        self.lbl2.place(x=0, y=0,relx=pox_x, rely=pox_y)

        self.t1 = threading.Thread(target=self.display_database, args=(self.insert_arr,self.insert_flag))
        self.t2 = threading.Thread(target=self.display_plate_img, args=(self.insert_img_arr,self.img_flag))
        self.t3 = threading.Thread(target=self.display_car_img)
        self.t1.start()
        self.t2.start()
        self.t3.start()

        window.mainloop()   # GUI꺼질때까지 기다림   

    def display_database(self,st,flag):
        while 1:
            time.sleep(2)
            if flag[0]==1:
                self.scrolled.insert(tkinter.INSERT,st[0]+'\n')
                self.scrolled.see(tkinter.END)
                flag[0]=0    
    def get_string(self,arg_str):
        self.insert_arr[0]=arg_str
        self.insert_flag[0]=1        #thread로 display_databas내의 if문을 검사중일 때 대입과정이 일어나서 같은 변수에 동시접근된다(-> 대입실패)? 아님.
    def display_plate_img(self,img,flag):
        while 1:
            if flag[0]==1:
                img2 = img[0]
                #img2 = cv2.resize(img2,dsize=(0,0),fx=1.7,fy=1.7)
                img2 = cv2.resize(img2,(400, 150), interpolation = cv2.INTER_CUBIC)
                img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
                img2 = Image.fromarray(img2)
                imgtk2 = ImageTk.PhotoImage(image=img2)

                self.lbl2.configure(image = imgtk2)

                flag[0]=0
    def display_car_img(self):
        captured_car_count=1
        pkg_path="/home/sdg/catkin_ws/src/image_saver"
        
        while 1:
            buffer=pkg_path+"/image/car_image/captured car"+str(captured_car_count)+".jpg"
            if os.path.exists(buffer):
                img1 = cv2.imread(buffer)
                #img1 = cv2.resize(img1,dsize=(0,0),fx=1.7,fy=1.7)
                img1 = cv2.resize(img1,(350, 350), interpolation = cv2.INTER_CUBIC)
                img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
                img1 = Image.fromarray(img1)
                imgtk1 = ImageTk.PhotoImage(image=img1)

                self.lbl1.configure(image = imgtk1)

                captured_car_count+=1
    def get_plate_img(self,plate):
        self.insert_img_arr[0]=plate
        self.img_flag[0]=1
