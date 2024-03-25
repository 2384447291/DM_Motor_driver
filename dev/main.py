#电机通讯的必要组件
from Communication import USB2CAN
from Motor import MotorMode
from Communication import global_CanMsgCenter
import Motor
import Planer
import thread
#你最爱的np
import numpy as np 
import time

#多线程组件
from threading import Thread
import threading


#挂墙时钟
t0 = 0.0
dt=0.0001

# 机器人参数和规划器
arg=np.mat([0,100,100,0,0,20.49])
n1=30000
n2=3000
pll=Planer.testlineplaner(Planer.fingerik(arg),n2)
rpll=Planer.resetplaner(pll.getJointAng(1),n1)

def Motor_control_thread(_interface):
    while True:
        # #虚假的1ms（暂时没有找到好的定时器方法）
        # time.sleep(0.001)
        #收发信息的驱动
        is_sending = global_CanMsgCenter.UpdateMassage(_interface)
        global_CanMsgCenter.RecieveMassage(_interface)   
#在这里写代码//不要动t0很危险
#-----------------------------------------------------------------------------------------------------------        
#-----------------------------------------------------------------------------------------------------------  
#-----------------------------------------------------------------------------------------------------------  
#-----------------------------------------------------------------------------------------------------------  
        #运动算法组件  
        global Motor1,Motor2,Motor3
        # print(time.time()-t0)
        i=(int)((time.time()-t0)/dt)
        
        # k= 8
        # d= 1.5

        if i>0 and i<=n1:
            k= 8
            d= 1.5
            Motor1.set(rpll.getJointAng(i)[0,0],0,k,d,0)
            Motor2.set(rpll.getJointAng(i)[0,1],0,k,d,0)
            Motor3.set(rpll.getJointAng(i)[0,2],0,k,d,0)
        else:# i>p:
            
            k=10
            d=1.1
            Motor1.set(pll.getJointAng(i-n1)[0,0],0,k,d,0)
            Motor2.set(pll.getJointAng(i-n1)[0,1],0,k,d,0)
            Motor3.set(pll.getJointAng(i-n1)[0,2],0,k,d,0)
#-----------------------------------------------------------------------------------------------------------        
#-----------------------------------------------------------------------------------------------------------  
#-----------------------------------------------------------------------------------------------------------  
#----------------------------------------------------------------------------------------------------------- 

if __name__ == '__main__' :
    #挂墙时间
    t0 = time.time()
    #定义USB
    m_USB = USB2CAN()
    #定义电机(注册4个以上电机UI会爆，懒得修了，你自己手动改吧)
    Motor1 = Motor.DMmotor(1,MotorMode["MIT_MODE"])
    Motor2 = Motor.DMmotor(2,MotorMode["MIT_MODE"])
    Motor3 = Motor.DMmotor(3,MotorMode["MIT_MODE"])
    #开启电机控制线程
    Motor_threading=threading.Thread(target=Motor_control_thread,args=[m_USB],name='Motor_control_thread')
    Motor_threading.daemon = True
    Motor_threading.start()    

    thread.draw_debugg_pic()

        