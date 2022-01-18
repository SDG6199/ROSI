import datetime

plate_arr_to_string_sum_count=[0]

def plate_arr_to_string_sum(arg_ar):
    ret=''
    temp=arg_ar
    if ' ' in temp:
            temp.remove(' ')
    for i in range(0,len(temp)):
        ret=ret+temp[i]
        if not temp[i].isdigit():    #(한글이면):  
            ret1=ret
            ret=''
        if i==len(temp)-1:                       #(끝나면):
            ret2=ret 

    plate_arr_to_string_sum_count[0]=plate_arr_to_string_sum_count[0]+1   
    ret=('%03d' % (plate_arr_to_string_sum_count[0]))+': '+ret1+' '+ret2+now.strftime(' %Y-%m-%d %H:%M:%S')
    return ret

arr=['0','1','1','가',' ','1','1','1']
now = datetime.datetime.now()

for i in range(0,100):
    print(plate_arr_to_string_sum(arr))

#print("['0','1']")
  
