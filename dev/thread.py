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
    global datalock,f,canvas,root
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

def draw_debugg_pic():
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