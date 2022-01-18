from PIL import Image
from pytesseract import *

def OCR(imgfile, lang='eng'):
    img=Image.open(imgfile)
    text=image_to_string(img, lang='Hangul')
    print(text)

buffer="/home/sdg/Desktop"+"/test_plate.jpg"
OCR(buffer)