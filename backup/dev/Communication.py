import serial
import serial.tools.list_ports
import numpy as np
import struct
import time
import queue
from enum import Enum
import sys

class Step(Enum):
    checking_head = 0
    recieve_cmd = 1
    processing_data = 2

def can_Baudrate():#1Mbps
    #                     帧头		索引	帧尾	
    send_data=np.array([0x55,0x05,0x00,0xAA,0x55],np.uint8)
    return send_data

def usart_Baudrate():#921600s
    #                   帧头        波特率			       数据位	 停止位	  校验位	 帧尾	
    send_data=np.array([0x55,0xAA,  0x00,0x00,0x00,0x00,  8,        1,       0,     0xAA,0x55],np.uint8)
    # print(send_data)
    #四字节小端序 0x00 0x0E 0x10 0x00 
    Baudrate=list(struct.pack("!i",921600))
    # Hex_baudrate = [hex(i) for i in Baudrate]
    # print(Hex_baudrate)
    for i in range(4):
        send_data[2+i]=Baudrate[i]
    return send_data     

class USB2CAN(object):  
    def __init__(self): 
        #                            帧头		 帧长	 命令	发送次数			   时间间隔				  ID类型 CAN ID	  帧类型   len	 idAcc	 dataAcc  data[len]		  CRC
        self.send_template=np.array([0x55,0xAA,  0x1e,  0x01,  0x01,0x00,0x00,0x00,  0x0a,0x00,0x00,0x00,  0x00,  0,0,0,0,  0x00,  0x08,  0x00,  0x00,  0,0,0,0,0,0,0,0,  0x88],np.uint8)
        ports_list = list(serial.tools.list_ports.comports())  
        plist = list(serial.tools.list_ports.comports())
        if len(plist) <= 0:
            print("The Serial port can't find!")
            print('program end')
            sys.exit(0) 
        else:
            for port in plist:
                uart_2_str = str(port)
                uart_name_list = uart_2_str.split()
                for j in uart_name_list:
                    if "串行设备" == j:
                        plist_0 = port
                        serialName = plist_0[0]
                        serialFd = serial.Serial(serialName,921600, timeout=60)
                        self.Serial = serialFd
                        print("check which port was really used >", serialFd.name,plist_0[1])
                        return
        print("The Serial port can't find!")            


    def CanSend(self,CAN_ID,Data):
        # Hex_Data = [hex(i) for i in Data]
        # print(Hex_Data)
        send_data=self.send_template
        if CAN_ID < 0xFF:
            send_data[13] = CAN_ID
        else:
            send_data[13] = CAN_ID >> 8
            send_data[14] = CAN_ID & 0xFF  
        send_data[21:29]=Data
        # Hex_send_data = [hex(i) for i in send_data]
        # print(Hex_send_data)
        self.Serial.write(bytes(send_data.T))    
    
    #调不调用无所谓
    # def Communication_connection(self):
    #     if(self.Serial.isOpen()):
    #         print("连接设备成功")
    #         self.Serial.write(bytes(usart_Baudrate().T))  
    #         self.Serial.write(bytes(can_Baudrate().T))              

class canMsg:
    def __init__(self,_ID,_data):
        self.ID = _ID
        self.data = _data

class CanMsgCenter():
    def __init__(self):
        self.SendMsgQue = queue.Queue()
        self.RecieveQue = queue.Queue()
        self.Recieve_step = Step["checking_head"]
        self.Thispacket = np.zeros((16,1),np.uint8) 
        self.ThispacketLength = 0
        self.Motordecitionary = {}
        self.registedmotor = []
        self.num_motor = 0

    def register(self,Motor):
        self.Motordecitionary[Motor.id] = Motor
        self.registedmotor.append(Motor)
        self.num_motor += 1
            
    def UpdateMassage(self,_interface):
         packettosend = 4
         for motor in self.registedmotor:
             motor.motorcontrol(motor.control_p_des,motor.control_v_des,motor.control_k_p,motor.control_k_d,motor.control_torque)   
         while not self.SendMsgQue.empty() and packettosend > 0:
             tempmsg = self.SendMsgQue.get()
             _interface.CanSend(tempmsg.ID,tempmsg.data)
             packettosend -= 1

    def RecieveMassage(self,_interface):  
        #接收消息
        data_length = _interface.Serial.inWaiting()
        if data_length > 0:
            data_bytes = _interface.Serial.read(data_length)
            data_np = np.frombuffer(data_bytes,dtype = np.uint8)
            # Hex_Data_np = [hex(i) for i in data_np]
            # print(Hex_Data_np)
            for processing_data in data_np:
                self.RecieveQue.put(processing_data)
        #处理消息      
        while not self.RecieveQue.empty():
            _data = self.RecieveQue.get()
            self.Thispacket[self.ThispacketLength] = _data
            self.ThispacketLength += 1
            if self.Recieve_step == Step["checking_head"]:
                if _data == 0xAA:
                   self.Recieve_step = Step["recieve_cmd"] 
                else:
                    self.Thispacket = np.zeros((16,1),np.uint8)    
                    self.ThispacketLength = 0    
            elif self.Recieve_step == Step["recieve_cmd"]:
                if _data == 0x01 or _data == 0x11 or _data == 0x02 or _data == 0x12:
                    self.Recieve_step = Step["processing_data"]
                elif _data == 0xEE:#直接停止
                    print("communication_error")
                    print('0xAA 0xEE') 
                    print(self.RecieveQue)
                    print('program end')
                    sys.exit(0)                                          
                else:
                    self.Recieve_step = Step["checking_head"]    
                    self.Thispacket = np.zeros((16,1),np.uint8)    
                    self.ThispacketLength = 0                  
            elif self.Recieve_step == Step["processing_data"]:             
                if self.ThispacketLength == 16: 
                    if _data == 0x55:
                        if self.Thispacket[1] == 0x01:
                            print("recieve_error")  
                        elif self.Thispacket[1] == 0x02:#如果没有对应的电机则直接死了
                            packet_id = self.Thispacket[3]
                            try:
                                self.Motordecitionary[packet_id[0]].resolve_feedback(0x02,self.Thispacket)
                            except KeyError:
                                print("Motor %d is not register" %packet_id)
                        elif self.Thispacket[1] == 0x11:
                            packet_id = self.Thispacket[7]
                            # try:
                            self.Motordecitionary[packet_id[0]].resolve_feedback(0x11,self.Thispacket)
                            # except KeyError:
                            #     print("Motor %d is not register" %packet_id)    
                    self.Recieve_step = Step["checking_head"]    
                    self.Thispacket = np.zeros((16,1),np.uint8)    
                    self.ThispacketLength = 0   
global_CanMsgCenter =  CanMsgCenter()  
