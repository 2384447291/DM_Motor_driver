#电机通讯的必要组件
from Communication import CanMsgCenter
from Communication import USB2CAN
from Motor import MotorMode
from Communication import global_CanMsgCenter
import Motor
import Planer
#你最爱的np
import numpy as np 

#绘图组件
import matplotlib
import matplotlib.pyplot as plt  
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from tkinter import *

#多线程组件
from threading import Thread,Lock
import threading
import time

#记录数据
import csv

#计算频率
frequencyA = 0
frequencyB = 0
frequencynum = 0
freq_every = 0

#挂墙时钟
t0 = 0.0
dt=0.001
last_sample_time = 0
have_head = False

# 机器人参数和规划器
arg=np.mat([0,0.100,0.100,0,0,-23.58/180*np.pi]) # l:[m], theta:[rad]
n1=3000
n2=3000
pll=Planer.testcatchplaner(Planer.finger_link(arg),n2)
rpll=Planer.resetplaner(pll.getJointAng(1),n1)

integral=0.0
err_last=0.0

#是否需要打开图像
is_need_plot = True  
is_need_check_freq = False
is_need_save_data = False
sample_time = 0.01

#定义电机(注册4个以上电机UI会爆，懒得修了，你自己手动改吧)
Motor1 = Motor.DMmotor(1,MotorMode["MIT_MODE"])
Motor2 = Motor.DMmotor(2,MotorMode["MIT_MODE"])
Motor3 = Motor.DMmotor(3,MotorMode["MIT_MODE"])

data_file_name = "aa1.csv"

def Motor_control_thread(_interface):
    while True:
        #在这里写代码//不要动t0很危险
        #-----------------------------------------------------------------------------------------------------------        
        #-----------------------------------------------------------------------------------------------------------  
        #-----------------------------------------------------------------------------------------------------------  
        #-----------------------------------------------------------------------------------------------------------  
        #运动算法组件  
        global Motor1,Motor2,Motor3
        i=(int)((time.time()-t0)/dt)

        if i>0 and i<=n1:
            k=5
            d=0.5
            Motor1.set(rpll.getJointAng(i)[0,0],0,k,d,0)
            Motor2.set(rpll.getJointAng(i)[0,1],0,k,d,0)
            Motor3.set(rpll.getJointAng(i)[0,2],0,k,d,0)
        else:# i>p:
            k=5
            d=0.5

            F_limit=3*np.mat([1,0,0])
            q_feedback=np.mat([Motor1.feedback_pos,Motor2.feedback_pos,Motor3.feedback_pos])
            w_feedback=np.mat([Motor1.feedback_vel,Motor2.feedback_vel,Motor3.feedback_vel])
            tao_feedback=np.mat([Motor1.feedback_torque,Motor2.feedback_torque,Motor3.feedback_torque])
            tao_limit=pll.k.isk(q_feedback,F_limit)
            
            if Planer.overlimit(tao_limit,tao_feedback)!=0:
                print("到阈值了")
                print(tao_limit)
                print(tao_feedback)


            Motor1.set(pll.getJointAng(i-n1)[0,0],(pll.getJointAng(i+1-n1)[0,0]-pll.getJointAng(i-n1)[0,0])/dt,k,d,0)
            Motor2.set(pll.getJointAng(i-n1)[0,1],(pll.getJointAng(i+1-n1)[0,1]-pll.getJointAng(i-n1)[0,1])/dt,k,d,0)
            Motor3.set(pll.getJointAng(i-n1)[0,2],(pll.getJointAng(i+1-n1)[0,2]-pll.getJointAng(i-n1)[0,2])/dt,k,d,0)








        # ## 单电机恒力输出测试=================================================================
        # ## 开环
        # global Motor1
        # i=(int)((time.time()-t0)/dt)
        # if i>1000:
        #     t_keep=0.4
        #     p_fb=Motor1.feedback_pos

        #     k=6
        #     d=0.5

        #     p_des=t_keep/k+p_fb
        #     Motor1.set(p_des,0,k,d,0)

        # # ## ====================================================================

        # ## 单电机恒力输出测试=================================================================
        # ## pid
        # global Motor1
        # i=(int)((time.time()-t0)/dt)
        # if i<=1000:
        #     integral=0.0
        #     err_last=0.0
        # elif i>1000:
        #     t_keep=0.4
        #     p_fb=Motor1.feedback_pos
        #     t_fb=Motor1.feedback_torque
        #     err=t_keep-t_fb
        #     pp=1
        #     ii=0.01
        #     dd=0.1

        #     integral+=err
        #     derivative=err-err_last
        #     t_ctr=pp*err+ii*integral+dd*derivative

        #     k=6
        #     d=0.5

        #     p_des=t_ctr/k+p_fb
        #     Motor1.set(p_des,0,k,d,0)

        #     err_last=err

        # # ## ====================================================================



        ## 速度规划测试=================================================================
        # if i>0 and i<=n1:
        #     k= 8
        #     d= 1
        #     # print(rpll.getJointAng(i))
        #     Motor1.set(rpll.getJointAng(i)[0,0],0,
        #                k,d,0)
        # else:# i>p:
        #     k=3nn
        #     d=1.2
        #     # print(rpll.getJointAng(i))
        #     # Motor1.set(pll.getJointAng(i-n1)[0,0],0,k,d,0)

        #     Motor1.set(pll.getJointAng(i-n1)[0,0],
        #                np.abs(pll.getJointAng(i-n1)[0,0]-pll.getJointAng(i+1-n1)[0,0])/dt,
        #                k,d,0)
        ## =================================================================











        #在这里写代码
        #-----------------------------------------------------------------------------------------------------------        
        #-----------------------------------------------------------------------------------------------------------  
        #-----------------------------------------------------------------------------------------------------------  
        #----------------------------------------------------------------------------------------------------------- 
       
       
       
       
       
       
       
       
       
        global frequencyB, frequencyA, freq_every, frequencynum 
        frequencyB = frequencyA
        frequencyA = time.time()
        if frequencyA != frequencyB:
            freq = 1/(frequencyA - frequencyB)
            frequencynum = frequencynum+1
            freq_every = freq_every + freq 
        freq = freq_every/frequencynum 
        if is_need_check_freq:
            print(freq)
        #收发信息的驱动
        is_sending = global_CanMsgCenter.UpdateMassage(_interface)
        global_CanMsgCenter.RecieveMassage(_interface) 
        global last_sample_time
        if is_need_save_data:
            global have_head
            if not have_head:
                header = ['time']
                number = 1
                for motor in global_CanMsgCenter.registedmotor:
                    str1 = 'M' + str(number) + ' ctrl_p'
                    str2 = 'M' + str(number) + ' fdb_pos'
                    str3 = 'M' + str(number) + ' ctrl_v'
                    str4 = 'M' + str(number) + ' fdb_v'
                    str5 = 'M' + str(number) + ' ctrl_tff'
                    str6 = 'M' + str(number) + ' fdb_tff'
                    header.append(str1)
                    header.append(str2)
                    header.append(str3)
                    header.append(str4)
                    header.append(str5)
                    header.append(str6)
                    number += 1
                with open(data_file_name, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(header)
                have_head = True
            if time.time()- t0 - last_sample_time > sample_time:
                with open(data_file_name,'a+') as f:
                    csv_write = csv.writer(f)
                    data_row = np.zeros((1,1),np.float64) 
                    data_row = time.time()-t0
                    for motor in global_CanMsgCenter.registedmotor:
                        data_row = np.append(data_row,[motor.control_p_des,motor.feedback_pos,
                                        motor.control_v_des,motor.feedback_vel,
                                        motor.control_torque,motor.feedback_torque])
                    csv_write.writerow(data_row)
                last_sample_time = time.time()- t0


#数据采样组件
def update_data():
    while True:
        time.sleep(0.01)
        global time_mem,data_mem,t0
        global global_CanMsgCenter
        #更新数据    
        #顺序为p_set,p_now,v_set,v_now,torque
        append_data = np.zeros((1,1),np.float64) 
        for motor in global_CanMsgCenter.registedmotor:
            append_data = np.append(append_data,[motor.control_p_des,motor.feedback_pos,
                                     motor.control_v_des,motor.feedback_vel,
                                     motor.control_torque,motor.feedback_torque])
        append_data = np.delete(append_data, 0) 
        append_data = np.reshape(append_data,(-1,1))
        datalock.acquire()
        time_mem = np.append(time_mem,time.time()-t0)
        data_mem = np.append(data_mem,append_data,axis=1)
        if time_mem.size > 500:#差不多10s500个数据
            time_mem = np.delete(time_mem, 0) 
            data_mem = np.delete(data_mem, 0, 1)   
        datalock.release()
        
def video_loop():  #动态图像现实窗口
    global time_mem,data_mem
    global global_CanMsgCenter
    datalock.acquire()
    temp_time_mem = time_mem
    temp_data_mem = data_mem
    datalock.release()
    f.clf()
    f.subplots_adjust(hspace=0.4, wspace = 0.4)
    plotlist = []
    for i in range(1,global_CanMsgCenter.num_motor+1):
        for j in range(1,4):
            temp_plot = f.add_subplot(global_CanMsgCenter.num_motor,3,i*3+j-3)
            plotlist.append(temp_plot)
    #绘图
    i = 0
    for a in plotlist:
        a.plot(temp_time_mem, temp_data_mem[i*2,:], color='red',linewidth=1.0)#实际
        a.plot(temp_time_mem, temp_data_mem[i*2+1,:], color='black',linewidth=1.0)#反馈
        a.tick_params(axis='x',
                 labelsize=4, # y轴字体大小设置
                  ) 
        a.tick_params(axis='y',
            labelsize=4, # y轴字体大小设置
            ) 
        a.set_xlim(xmin = temp_time_mem[0], xmax = temp_time_mem[-1])
        i_title = i%3
        if  i_title == 0:
            a.set_title('Motor{}Pos'.format(i//3), fontsize=6)
        elif i_title == 1:
            a.set_title('Motor{}Vec'.format(i//3), fontsize=6)
        elif i_title == 2:
            a.set_title('Motor{}Torque'.format(i//3), fontsize=6) 
        i+=1  
    canvas.draw()
    root.after(10, video_loop)

if __name__ == '__main__' :
    #挂墙时间
    t0 = time.time()
    #定义USB
    m_USB = USB2CAN()
    if is_need_plot:
        #开启电机控制线程
        Motor_threading=threading.Thread(target=Motor_control_thread,args=[m_USB],name='Motor_control_thread')
        Motor_threading.daemon = True
        Motor_threading.start()   
        #---------------------------------------------------画图----------------------------------------------
        #开启数据更新线程
        Data_update_threading=threading.Thread(target=update_data,name='Update_data_thread')
        Data_update_threading.daemon = True
        Data_update_threading.start()      
        #绘图主线程
        data_mem = np.zeros((global_CanMsgCenter.num_motor*6,1),np.float64) 
        time_mem = np.zeros((1,1),np.float64) 
        datalock = Lock()
        root = Tk()
        root.title("Set title")
        root.geometry('1000x500')
        
        """
        图像画布设置
        """
        panel = Label(root)  # initialize image panel
        panel.place(x=0,y=0,anchor='nw')
        root.config(cursor="arrow")
        
        matplotlib.use('TkAgg')
        f = Figure(figsize=(7, 5/(5-global_CanMsgCenter.num_motor)), dpi=130)
        canvas = FigureCanvasTkAgg(f, master=root)
        canvas.draw()
        canvas.get_tk_widget().place(x=0,y=0 ,anchor='nw')
        video_loop()
        root.mainloop()
    else:
        Motor_control_thread(m_USB)
        