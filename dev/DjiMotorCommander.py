#最丑陋的一集

import serial
import serial.tools.list_ports
import numpy as np
import struct
import time
import queue
from enum import Enum
import sys

def dJI_LIMIT_MIN_MAX(x,min,max):
    if x<=min:
        x=min
    elif x>max:
        x=max

Bit_MIN = -16384
Bit_MAX = 16384

Current_MIN = -20
Current_MAX = 20

class DjiStep(Enum):
    Djichecking_head = 0
    Djirecieve_cmd = 1
    Djiprocessing_data = 2

class DjicanMsg:
    def __init__(self,_ID,_data):
        self.ID = _ID
        self.data = _data

def float_to_int(x):
        span_bit = Bit_MAX - Bit_MIN
        span_current = Current_MAX - Current_MIN
        return np.int16(x/span_current*span_bit)

class DjiMsgCenter():
    def __init__(self):
        self.SendMsgQue = queue.Queue()
        self.RecieveQue = queue.Queue()
        self.Recieve_step = DjiStep["Djichecking_head"]
        self.Thispacket = np.zeros((16,1),np.uint8) 
        self.ThispacketLength = 0
        self.Motordecitionary = {}
        self.registedmotor = []
        self.num_motor = 0
        self.need_send = False
        self.currentlist = np.zeros((4,1),np.float32) 

    def register(self,Motor):
        self.Motordecitionary[0x200 + Motor.id] = Motor
        self.registedmotor.append(Motor)
        self.num_motor += 1

    def packcurrent(self):
            for current_set in self.currentlist:
                dJI_LIMIT_MIN_MAX(current_set,  Current_MIN,  Current_MAX)
            buf = np.zeros((8,1),np.uint8)  

            current_bit_1 = float_to_int(self.currentlist[0])  
            current_bit_2 = float_to_int(self.currentlist[1])
            current_bit_3 = float_to_int(self.currentlist[2]) 
            current_bit_4 = float_to_int(self.currentlist[3])

            buf[0] = current_bit_1 >> 8
            buf[1] = current_bit_1 & 0xFF
            buf[2] = current_bit_2 >> 8
            buf[3] = current_bit_2 & 0xFF
            buf[4] = current_bit_3 >> 8
            buf[5] = current_bit_3 & 0xFF
            buf[6] = current_bit_4 >> 8
            buf[7] = current_bit_4 & 0xFF
            tempmsg = DjicanMsg(0x200,buf.reshape(8,))
            global_DjiMsgCenter.SendMsgQue.put(tempmsg)

    def UpdateMassage(self,_interface):
         packettosend = 4
         for motor in self.registedmotor:
             motor.updatefeedback()#传感器
             motor.update()#控制器
         self.packcurrent()#执行器
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
            if self.Recieve_step == DjiStep["Djichecking_head"]:
                if _data == 0xAA:
                   self.Recieve_step = DjiStep["Djirecieve_cmd"] 
                else:
                    self.Thispacket = np.zeros((16,1),np.uint8)    
                    self.ThispacketLength = 0    
            elif self.Recieve_step == DjiStep["Djirecieve_cmd"]:
                if _data == 0x01 or _data == 0x11 or _data == 0x02 or _data == 0x12:
                    self.Recieve_step = DjiStep["Djiprocessing_data"]
                elif _data == 0xEE:#直接停止
                    print("communication_error")
                    print('0xAA 0xEE') 
                    print(self.RecieveQue)
                    print('program end')
                    sys.exit(0)                                          
                else:
                    self.Recieve_step = DjiStep["Djichecking_head"]    
                    self.Thispacket = np.zeros((16,1),np.uint8)    
                    self.ThispacketLength = 0                  
            elif self.Recieve_step == DjiStep["Djiprocessing_data"]:             
                if self.ThispacketLength == 16: 
                    if _data == 0x55:
                        if self.Thispacket[1] == 0x01:
                            print("recieve_error") 
                        elif self.Thispacket[1] == 0x02:#发送失败
                            print('send fail')
                            message_buffer = self.Thispacket[7:15]
                            Hex_Thispacket = [hex(i) for i in message_buffer.reshape(8,)]
                            print(Hex_Thispacket)


                            # message_buffer = self.Thispacket[0:15]
                            # Hex_Thispacket = [hex(i) for i in message_buffer.reshape(15,)]
                            # print(Hex_Thispacket)
                        elif self.Thispacket[1] == 0x11:
                            #print('recieve succcess')
                            message_buffer = self.Thispacket[7:15]
                            Hex_Thispacket = [hex(i) for i in message_buffer.reshape(8,)]
                            # print(Hex_Thispacket)
                            packet_id = struct.unpack("I",self.Thispacket[3:7])
                            try:
                                self.Motordecitionary[packet_id[0]].resolve_feedback(message_buffer)
                            except KeyError:
                                print("Motor %d is not register" %packet_id)  
                    self.Recieve_step = DjiStep["Djichecking_head"]    
                    self.Thispacket = np.zeros((16,1),np.uint8)    
                    self.ThispacketLength = 0   
global_DjiMsgCenter =  DjiMsgCenter()  
