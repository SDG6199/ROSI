import cv2
import tkinter 
from PIL import ImageTk, Image
from tkinter import ttk    # import tkinter 와 다른 명령이다.
from tkinter import scrolledtext
from tkinter import filedialog  # @@ tkinter.filedialog

window=tkinter.Tk()
window.title("test")
window.geometry("940x600+500+500")  # w,h x,y
window.resizable(False, False)       # w,h

## radio button 
def radCall():
    radSel = radVar.get()
    if radSel == 1:
        window.configure(background=COLOR1)
    elif radSel == 2:
        window.configure(background=COLOR2)
    elif radSel == 3:
        window.configure(background=COLOR3)

COLOR1 = "Blue"
COLOR2 = "Gold"
COLOR3 = "Red"

radVar = tkinter.IntVar()

rad1 = tkinter.Radiobutton(window, text=COLOR1, variable=radVar, value=1, command=radCall)
rad1.grid(column=0, row=4, sticky=tkinter.W, columnspan=3)

rad2 = tkinter.Radiobutton(window, text=COLOR2, variable=radVar, value=2, command=radCall)
rad2.grid(column=1, row=4, sticky=tkinter.W, columnspan=3)

rad3 = tkinter.Radiobutton(window, text=COLOR3, variable=radVar, value=3, command=radCall)
rad3.grid(column=2, row=4, sticky=tkinter.W, columnspan=3)


# Entry 
name = tkinter.StringVar()
name_entered = ttk.Entry(window, width=12, textvariable=name)
name_entered.grid(column=0, row=1)
name_entered.focus()   # GUI 가 실행되면 Entry에 바로 활성화.

# text box (Choose a number)  
ttk.Label(window, text="Choose a number:").grid(column=1, row=0)

# combo box
number = tkinter.StringVar()
number_chosen = ttk.Combobox(window, width=12, textvariable=number, state='readonly')
number_chosen['values'] = (1, 2, 4, 42, 100)
number_chosen.grid(column=1, row=1)
number_chosen.current(4)  #해당 index의 목록 표시

def click_me():
    action.configure(text='Hello ' + name.get() + ' ' + number_chosen.get())
    
# Button (Click Me)  
action = ttk.Button(window, text="Click Me!", command=click_me)
action.grid(column=2, row=1)  

# Checkbutton (Disabled)  state 또한 'disabled'.
chVarDis = tkinter.IntVar()
check1 = tkinter.Checkbutton(window, text="Disabled", variable=chVarDis, state='disabled')
check1.select()    #체크 상태
check1.grid(column=0, row=3, sticky='e')  # w는 tkinter.W로도 쓸 수 있다.

# scrollbar 
frame=tkinter.Frame(window)
scrollbar=tkinter.Scrollbar(frame)
scrollbar.pack(side="right", fill="y")

listbox=tkinter.Listbox(frame, yscrollcommand = scrollbar.set)  #리스트박스의 세로스크롤 위젯 적용
for line in range(1,1001):
   listbox.insert(line, str(line) + "/1000")
listbox.pack(ipadx=5)

scrollbar["command"]=listbox.yview
frame.pack()

window.mainloop()  

