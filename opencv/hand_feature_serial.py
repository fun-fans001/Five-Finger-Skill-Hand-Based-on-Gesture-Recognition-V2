import mediapipe as mp
import cv2
import numpy as np
import serial 
import time
import serial.tools.list_ports
 

def get_angle(v1,v2):
    angle = np.dot(v1,v2)/(np.sqrt(np.sum(v1*v1))*np.sqrt(np.sum(v2*v2)))
    angle = np.arccos(angle)/3.14*180
    return angle    
    
def get_str_guester(up_fingers,list_lms):
    
    if len(up_fingers)==1 and up_fingers[0]==8:
        
        v1 = list_lms[6]-list_lms[7]
        v2 = list_lms[8]-list_lms[7]
        
        angle = get_angle(v1,v2)
       
        if angle<160:
            str_guester = "9"
        else:
            str_guester = "1"
    
    elif len(up_fingers)==1 and up_fingers[0]==4:
        str_guester = "Good"
    
    elif len(up_fingers)==2 and up_fingers[0]==8 and up_fingers[1]==12:
        str_guester = "2"
        
    elif len(up_fingers)==2 and up_fingers[0]==4 and up_fingers[1]==20:
        str_guester = "6"
        
    elif len(up_fingers)==2 and up_fingers[0]==4 and up_fingers[1]==8:
        str_guester = "8"
    
    elif len(up_fingers)==3 and up_fingers[0]==8 and up_fingers[1]==12 and up_fingers[2]==16:
        str_guester = "3"
    
    elif len(up_fingers)==3 and up_fingers[0]==4 and up_fingers[1]==8 and up_fingers[2]==12:
  
        dis_8_12 = list_lms[8,:] - list_lms[12,:]
        dis_8_12 = np.sqrt(np.dot(dis_8_12,dis_8_12))
        
        dis_4_12 = list_lms[4,:] - list_lms[12,:]
        dis_4_12 = np.sqrt(np.dot(dis_4_12,dis_4_12))
        
        if dis_4_12/(dis_8_12+1) <3:
            str_guester = "7"
        
        elif dis_4_12/(dis_8_12+1) >5:
            str_guester = "Gun"
        else:
            str_guester = "7"
            
    elif len(up_fingers)==3 and up_fingers[0]==4 and up_fingers[1]==8 and up_fingers[2]==20:
        str_guester = "ROCK"
    
    elif len(up_fingers)==4 and up_fingers[0]==8 and up_fingers[1]==12 and up_fingers[2]==16 and up_fingers[3]==20:
        str_guester = "4"
    
    elif len(up_fingers)==5:
        str_guester = "5"
        
    elif len(up_fingers)==0:
        str_guester = "10"
    
    else:
        str_guester = " "

    return str_guester
        
        

if __name__ == "__main__":
   
    cap = cv2.VideoCapture(0)
    # 定义手 检测对象
    mpHands = mp.solutions.hands
    hands = mpHands.Hands()
    mpDraw = mp.solutions.drawing_utils
   # 读取串口列表
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0:
        print("无串口设备")
    else:
        print("可用的串口设备如下: ")
        print("%-10s %-30s %-10s" % ("num", "name", "number"))
        for i in range(len(ports_list)):
            comport = list(ports_list[i])
            comport_number, comport_name    =    comport[0], comport[1]
            print("%-10s %-30s %-10s" % (i, comport_name, comport_number))
 
        # 打开串口
        port_num = ports_list[0][0]
        print("默认选择串口: %s" % port_num)
        # 串口号: port_num, 波特率: 115200, 数据位: 7, 停止位: 2, 超时时间: 0.5秒 115200
        # ser = serial.Serial(port=port_num, baudrate= 9600, bytesize=serial.SEVENBITS, stopbits=serial.STOPBITS_TWO,
        #                     timeout=0.5)
        # 串口号: port_num, 波特率: 9600/115200, 数据位: 8, 停止位: 1, 超时时间: 0.5秒  9600
        ser = serial.Serial(port=port_num, baudrate= 9600,  bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE    ,
                            timeout=0.5)

        if not ser.isOpen():
            print("打开串口失败")
        else:
            print("打开串口成功, 串口号: %s" % ser.name)
   ####--initial 
        str_guester       =      0;
        time_cnt          =      0;    
        flag_num1=0,
        flag_num2=0,
        flag_num3=0,
        flag_num4=0,
        flag_num5=0;
    
    while True:

        # 读取一帧图像
        success, img = cap.read()
        if not success:
            continue
        image_height, image_width, _ = np.shape(img)
        
        # 转换为RGB
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # 得到检测结果
        results = hands.process(imgRGB)
        
        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]
            
            mpDraw.draw_landmarks(img,hand,mpHands.HAND_CONNECTIONS)
            
            # 采集所有关键点的坐标
            list_lms = []    
            for i in range(21):
                pos_x = hand.landmark[i].x*image_width
                pos_y = hand.landmark[i].y*image_height
                list_lms.append([int(pos_x),int(pos_y)])
            
            # 构造凸包点
            list_lms = np.array(list_lms,dtype=np.int32)
            hull_index = [0,1,2,3,6,10,14,19,18,17,10]
            hull = cv2.convexHull(list_lms[hull_index,:])
            # 绘制凸包
            cv2.polylines(img,[hull], True, (0, 255, 0), 2)
                
            # 查找外部的点数
            n_fig = -1
            ll = [4,8,12,16,20] 
            up_fingers = []
            
            for i in ll:
                pt = (int(list_lms[i][0]),int(list_lms[i][1]))
                dist= cv2.pointPolygonTest(hull,pt,True)
                if dist <0:
                    up_fingers.append(i)
            
            # print(up_fingers)
            # print(list_lms)
            # print(np.shape(list_lms))
            str_guester = get_str_guester(up_fingers,list_lms)
            
            cv2.putText(img,' %s'%(str_guester),(90,90),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,0),4,cv2.LINE_AA)
            print("str_guester: ",str_guester)    
######################--------------------fps---------------------------################
            # prev_time       =     0
            # frame_count     =     0
            # # 计算当前时间和帧数
            # current_time = time.time()
            # frame_count += 1 
            # # 如果足够时间已过，计算并显示帧率
            # if current_time - prev_time >= 1:
            #     fps = frame_count / (current_time - prev_time)
            #     prev_time       =    current_time
            #     frame_count     =   0
            #     # 将帧率转换为字符串
            #     fps_text = "FPS: {:.2f}".format(fps)
        
            #     # 在图像上绘制文本
            #     cv2.putText(img, fps_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)     
                        
            # for i in ll:
            #     pos_x = hand.landmark[i].x*image_width
            #     pos_y = hand.landmark[i].y*image_height
            #     # 画点
            #     cv2.circle(img, (int(pos_x),int(pos_y)), 3, (0,255,255),-1)
                            
        cv2.imshow("hands",img)
    #######--------------------serial-send--------------------###
 
        if str_guester == "1":
            data  =   1   
            time.sleep(1)  
            print("发送数据: %X" % data)
            write_len = ser.write(bytearray([data]))
            print("串口发出{}个字节".format(write_len))
  
        if str_guester == "2":
            data  =   2    
            time.sleep(1)                    
            print("发送数据: %X" % data)
            write_len = ser.write(bytearray([data])) 
            print("串口发出{}个字节".format(write_len))
        if str_guester == "3":
            data  =  3 
            time.sleep(1)         
            print("发送数据: %X" % data)
            write_len = ser.write(bytearray([data]))
            print("串口发出{}个字节".format(write_len))
        if str_guester == "4":
            data  =  4
            print("发送数据: %X" % data)
            write_len = ser.write(bytearray([data]))
            print("串口发出{}个字节".format(write_len))

        if str_guester == "5":
            data  =  5
            print("发送数据: %X" % data)
            write_len = ser.write(bytearray([data]))
            print("串口发出{}个字节".format(write_len))
        if str_guester == "6":
            data  =  6
            print("发送数据: %X" % data)
            write_len = ser.write(bytearray([data]))
            print("串口发出{}个字节".format(write_len))

 
            # 关闭串口
        # ser.close()
        # if ser.isOpen():
        #     print("串口未关闭")
        # else:
        #     print("串口已关闭")

        key =  cv2.waitKey(1) & 0xFF   

        # 按键 "q" 退出
        if key ==  ord('q'):
            break
    cap.release() 
       
    
    
    
    
    
    
    
    
    
    
    
    
    