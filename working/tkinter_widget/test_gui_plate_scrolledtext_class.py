import cv2
import tkinter 
from PIL import ImageTk, Image
from tkinter import ttk    # import tkinter 와 다른 명령이다.
from tkinter import scrolledtext
from tkinter import filedialog  # @@ tkinter.filedialog
import threading
import time
import tkinter.font as tkFont
import numpy as np

class GUI:
    def __init__(self):
        self.insert_flag=[0]
        self.insert_arr=["#Illegal parking car list"]   # mutable object
        self.insert_img_arr=[0]
        self.img_flag=[0]
        self.count=0

        self.t1 = threading.Thread(target=self.display_database, args=(self.insert_arr,self.insert_flag))
        self.t2 = threading.Thread(target=self.display_plate_img, args=(self.insert_img_arr,self.img_flag))
        self.t3 = threading.Thread(target=self.get_string, args=("hi",))
        self.t4 = threading.Thread(target=self.get_plate_img)
        self.t1.start()
        self.t2.start()
        self.t3.start()
        self.t4.start()
        print("thread_display_database start.")

        window=tkinter.Tk()
        window.title("GUI")
        window.geometry("1920x1080+500+500")     # w,h x,y
        window.resizable(True, True)             # w,h
        C = tkinter.Canvas(window, bg="chocolate1", height=1080, width=1920)  #1. pale goldenrod 2.navajo white 3.chocolate1
        C.pack()
        self.fontStyle_title = tkFont.Font(family="Lucida Grande", size=50)  #Helvetica
        self.fontStyle_text = tkFont.Font(family="Lucida Grande", size=75)
        ####
        ##  Captured Car     
        pox_x=0.08; pox_y=0.13
        ttk.Label(window, text="의심차량", font=self.fontStyle_title).place(relx=pox_x,rely=pox_y*0.4)

        img1 =cv2.imread("captured car1.jpg")
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
        img1 = Image.fromarray(img1)

        imgtk1 = ImageTk.PhotoImage(image=img1)
        #imgtk1 = ImageTk.PhotoImage(Image.open("captured car1.jpg"))   #image나
        #imgtk1 = tkinter.PhotoImage(file='captured car1.jpg')          #file로도 가능.
        self.lbl1 = tkinter.Label(window, image=imgtk1)
        self.lbl1.place(relx=pox_x, rely=pox_y)
       
        ## Illegal parking car list
        pox_x=0.37; pox_y=0.48
        ttk.Label(window, text="단속차량 번호", font=self.fontStyle_title).place(relx=pox_x,rely=pox_y*0.8)

        scrolled_w = 17; scrolled_h = 4  # w, h
        self.scrolled = scrolledtext.ScrolledText(window, width=scrolled_w, height=scrolled_h, font=self.fontStyle_text,wrap=tkinter.WORD)   # 단어별로 자동개행.
        self.scrolled.place(relx=pox_x,rely=pox_y)

        ## Captured Plate
        pox_x=0.37; pox_y=0.13
        ttk.Label(window, text="번호판", font=self.fontStyle_title).place(relx=pox_x,rely=pox_y*0.4)

     #   img2 = np.zeros((2,2,3),dtype=int)  #cv2.imread("test_plate.jpg")
     #   img2 = cv2.resize(img2,dsize=(0,0),fx=1.1,fy=1.1)
     #   img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
     #   img2 = Image.fromarray(img2)

     #   imgtk2 = ImageTk.PhotoImage(image=img2)
        #imgtk2 = ImageTk.PhotoImage(Image.open("test_plate.jpg"))   #image나
        #imgtk2 = tkinter.PhotoImage(file='captured car2.jpg')          #file로도 가능.
     #   lbl2 = tkinter.Label(window, image=imgtk2)
     #   lbl2.place(x=0, y=0,relx=pox_x, rely=pox_y)

        window.mainloop()   # GUI꺼질때까지 기다림

    def display_database(self,st,flag):
        while 1:
            time.sleep(2)
            if flag[0]==1:
                self.scrolled.insert(tkinter.INSERT,st[0]+'\n')
                self.scrolled.see(tkinter.END)
                flag[0]=0    
    def get_string(self,arg_str):
        while 1:
            self.count=self.count+1
            self.insert_arr[0]=arg_str+str(self.count)
            self.insert_flag[0]=1 
            time.sleep(0.5)
    def display_plate_img(self,img,flag):
        while 1:
            if flag[0]==1:
                img1 = img[0]
                img1 = cv2.resize(img1,dsize=(0,0),fx=1.7,fy=1.7)
                img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
                img1 = Image.fromarray(img1)
                imgtk1 = ImageTk.PhotoImage(image=img1)

                self.lbl1.configure(image = imgtk1)

                flag[0]=0
    def get_plate_img(self):
        while 1:
            self.insert_img_arr[0] =cv2.imread("captured car1.jpg")  
            self.img_flag[0]=1
            time.sleep(0.5)
            self.insert_img_arr[0] =cv2.imread("test_plate.jpg")  
            self.img_flag[0]=1
            time.sleep(0.5) 

gui=GUI()
