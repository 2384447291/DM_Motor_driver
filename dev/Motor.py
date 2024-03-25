import numpy as np
from enum import Enum
from Communication import canMsg
from Communication import CanMsgCenter
from Communication import global_CanMsgCenter

#电机上限参数
P_MIN= -12.5   
P_MAX= 12.5       
V_MIN=-30.0  
V_MAX= 30.0
KP_MIN=0.0     
KP_MAX=500.0
KD_MIN=0.0     
KD_MAX=5.0
T_MIN=-10.0
T_MAX=10.0

class MotorMode(Enum):
    MIT_MODE = 0#P_des：位置给定//V_des：速度给定//Kp：位置比例系数//Kd：位置微分系数//T_ff：转矩给定值
    POS_VEC_MODE = 1#P_des：位置给定，V_des：速度给定
    VEC_MODE =2#V_des：速度给定

class MotorStatus(Enum):
    Safe = 0#ok
    overvoltage = 1#超压
    Undervoltage = 2#欠压
    overelectric = 3#过电流
    MosOverHeat = 4#MOS过热
    coilOverHeat= 5#电机线圈过温
    CommunicationLost = 6#通讯丢失
    overload = 7#过载
    

def float_to_uint(x,x_min, x_max,bits):
    if bits <= 16:
        span=x_max-x_min
        offset=x_min
        return np.uint16((x-offset)*((1<<bits)-1)/span)
    elif bits > 16:
        span=x_max-x_min
        offset=x_min
        return np.uint32((x-offset)*((1<<bits)-1)/span)

def uint_to_float(x_int,x_min, x_max,bits):
    span=x_max-x_min
    offset=x_min
    return float(x_int*span/((1<<bits)-1)+offset)
def LIMIT_MIN_MAX(x,min,max):
    if x<=min:
        x=min
    elif x>max:
        x=max

class DMmotor(object):  
    def __init__(self,ID,_Motor_mode): 
        self.MotorMode = _Motor_mode
        self.MotorStatus = MotorStatus["Safe"]
        self.is_enable = False
        self.id = ID
        self.feedback_pos = float(0)
        self.feedback_vel = float(0)
        self.feedback_torque = float(0)
        global_CanMsgCenter.register(self)
        self.disable_motor()
        self.enable_motor()
        self.save_zero()
        self.control_p_des = 0.0
        self.control_v_des = 0.0
        self.control_k_p = 0.0
        self.control_k_d = 0.0
        self.control_torque = 0.0

    def enable_motor(self):
        global global_CanMsgCenter
        buf = np.array([self.id,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc],np.uint8)
        tempmsg = canMsg(buf[0],buf[1:9])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)
        self.is_enable = True

    def disable_motor(self):
        global global_CanMsgCenter  
        buf = np.array([self.id,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd],np.uint8)
        tempmsg = canMsg(buf[0],buf[1:9])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)   
        self.is_enable = False         

    def save_zero(self):
        global global_CanMsgCenter  
        buf = np.array([self.id,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe],np.uint8)
        tempmsg = canMsg(buf[0],buf[1:9])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)    

    def clear_error(self):    
        global global_CanMsgCenter  
        buf = np.array([self.id,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfb],np.uint8)
        tempmsg = canMsg(buf[0],buf[1:9])
        global_CanMsgCenter.SendMsgQue.put(tempmsg)

    def motorcontrol(self,_p_des,_v_des,_kp=0,_kd=0,_tff=0):
        global global_CanMsgCenter  
        if self.MotorMode == MotorMode["MIT_MODE"]:
            LIMIT_MIN_MAX(_p_des,  P_MIN,  P_MAX)
            LIMIT_MIN_MAX(_v_des,  V_MIN,  V_MAX)
            LIMIT_MIN_MAX(_kp, KP_MIN, KP_MAX)
            LIMIT_MIN_MAX(_kd, KD_MIN, KD_MAX)
            LIMIT_MIN_MAX(_tff,  T_MIN,  T_MAX)
            p = float_to_uint(_p_des, P_MIN, P_MAX, 16)            
            v = float_to_uint(_v_des, V_MIN, V_MAX, 12)
            kp = float_to_uint(_kp, KP_MIN, KP_MAX, 12)
            kd = float_to_uint(_kd, KD_MIN, KD_MAX, 12)
            t = float_to_uint(_tff, T_MIN, T_MAX, 12)
            buf=np.zeros((9,1),np.uint8)     
            buf[0] = self.id
            buf[1] = p>>8
            buf[2] = p&0xFF
            buf[3] = v>>4
            buf[4] = ((v&0xF)<<4)|(kp>>8)
            buf[5] = kp&0xFF
            buf[6] = kd>>4
            buf[7] = ((kd&0xF)<<4)|(t>>8)
            buf[8] = t&0xff
            tempmsg = canMsg(buf[0],buf[1:9].reshape(8,))
            global_CanMsgCenter.SendMsgQue.put(tempmsg)  
    def set(self,_p_des,_v_des,_kp=0,_kd=0,_tff=0):
        self.control_p_des = _p_des
        self.control_v_des = _v_des
        self.control_k_p = _kp
        self.control_k_d = _kd
        self.control_t = _tff  

# class MotorStatus(Enum):
#     Safe = 0#ok
#     overvoltage = 1#超压
#     Undervoltage = 2#欠压
#     overelectric = 3#过电流
#     MosOverHeat = 4#MOS过热
#     coilOverHeat= 5#电机线圈过温
#     CommunicationLost = 6#通讯丢失
#     overload = 7#过载
    def resolve_feedback(self,cmd,packet):
        if cmd == 0x11:#接收成功
            packet_status = (packet[7]>>4)&0xF
            if packet_status == 0x08:
                self.MotorStatus = MotorStatus["overvoltage"]
            elif packet_status == 0x09:
                self.MotorStatus = MotorStatus["Undervoltage"]
            elif packet_status == 0x0A:
                self.MotorStatus = MotorStatus["overelectric"]
            elif packet_status == 0x0B:
                self.MotorStatus = MotorStatus["MosOverHeat"]
            elif packet_status == 0x0C:
                self.MotorStatus = MotorStatus["coilOverHeat"]
            elif packet_status == 0x0D:
                self.MotorStatus = MotorStatus["CommunicationLost"]
            elif packet_status == 0x0E:
                self.MotorStatus = MotorStatus["overload"]   
            else:
                self.MotorStatus = MotorStatus["Safe"]       
            packet_pos = np.uint16(0)                
            packet_pos = np.uint16(packet[8])<<8 | packet[9] 
            packet_vec = np.uint16(0)
            packet_vec = np.uint16(packet[10])<<4 | packet[11]>>4
            packet_torque = np.uint16(0)
            packet_torque = np.uint16((packet[11]&0xF))<<8 | packet[12]
            self.feedback_pos = uint_to_float(packet_pos,P_MIN,P_MAX,16)
            self.feedback_vel = uint_to_float(packet_vec,V_MIN,V_MAX,12)
            self.feedback_torque = uint_to_float(packet_torque,T_MIN,T_MAX,12)  
            # print('Feedback: Motor %d move with Pos: %f, Vec: %f, Torque: %f' %(self.id,self.feedback_pos,self.feedback_vel,self.feedback_torque))          

        elif cmd == 0x02:#发送失败 
            if packet[7] == 0xFF and  packet[8] == 0xFF and  packet[9] == 0xFF and packet[10] == 0xFF and packet[11] == 0xFF and  packet[12] == 0xFF and packet[13] == 0xFF:
                if packet[14] == 0xFC:
                    print('send error Motor %d can not enter' %(self.id))
                elif packet[14] == 0xFD:
                    print('send error Motor %d can not out' %(self.id))
                elif packet[14] == 0xFE:
                    print('send error Motor %d can not save zero' %(self.id))          
                elif packet[14] == 0xFB:
                    print('send error Motor %d can not clear error' %(self.id))
            else:
                packet_pos = np.uint16(0)                
                packet_pos = np.uint16(packet[7])<<8 | packet[8] 
                packet_vec = np.uint16(0)
                packet_vec = np.uint16(packet[9])<<4 | packet[10]>>4
                packet_kp = np.uint16(0)
                packet_kp = np.uint16((packet[10]&0xF))<<8 | packet[11]
                packet_kd = np.uint16(0)
                packet_kd = np.uint16(packet[12])<<4 | packet[13]>>4     
                packet_T = np.uint16(0)
                packet_T = np.uint16((packet[13]&0xF))<<8 | packet[14]                
                packet_pos = uint_to_float(packet_pos,P_MIN,P_MAX,16)
                packet_vec = uint_to_float(packet_vec,V_MIN,V_MAX,12)
                packet_kp = uint_to_float(packet_kp,KP_MIN,KP_MAX,12)
                packet_kd = uint_to_float(packet_kd,KD_MIN,KD_MAX,12)
                packet_T = uint_to_float(packet_T,T_MIN,T_MAX,12)  

                print('send error Motor %d move with Pos: %f, Vec: %f, Kp: %f, Kd: %f, Torque: %f' %(self.id,packet_pos,packet_vec,packet_kp,packet_kd,packet_T))          
            
        #有bug但是估计用不到
        # elif self.MotorMode == MotorMode["POS_VEC_MODE"]:    
        #     LIMIT_MIN_MAX(_p_des,  P_MIN,  P_MAX)
        #     LIMIT_MIN_MAX(_v_des,  V_MIN,  V_MAX)
        #     p = float_to_uint(_p_des, P_MIN, P_MAX, 32)            
        #     v = float_to_uint(_v_des, V_MIN, V_MAX, 32)
        #     buf=np.zeros((9,1),np.uint8)     
        #     tempid = self.id + 0x100
        #     print(p)
        #     buf[1] = p>>24
        #     buf[2] = (p>>16)&0xFF
        #     buf[3] = (p>>8)&0xFF
        #     buf[4] = p&0xFF
        #     buf[5] = v>>24
        #     buf[6] = (v>>16)&0xFF
        #     buf[7] = (v>>8)&0xFF
        #     buf[8] = v&0xFF
        #     tempmsg = canMsg(tempid,buf[1:9])
        #     global_CanMsgCenter.SendMsgQue.put(tempmsg)               
        # elif self.MotorMode == MotorMode["VEC_MODE"]:    
        #     LIMIT_MIN_MAX(_p_des,  P_MIN,  P_MAX)
        #     LIMIT_MIN_MAX(_v_des,  V_MIN,  V_MAX)
        #     p = float_to_uint(_p_des, P_MIN, P_MAX, 32)            
        #     v = float_to_uint(_v_des, V_MIN, V_MAX, 32)
        #     tempid = self.id + 0x100
        #     buf[1] = v>>24
        #     buf[2] = (v>>16)&0xFF
        #     buf[3] = (v>>8)&0xFF
        #     buf[4] = v&0xFF
        #     tempmsg = canMsg(tempid,buf[1:9])
        #     global_CanMsgCenter.SendMsgQue.put(tempmsg)  
          
        