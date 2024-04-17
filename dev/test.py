import csv
import numpy as np
import serial
import serial.tools.list_ports
import numpy as np
import struct
import time
import queue
from enum import Enum
import sys
# buf=np.zeros((9,1),np.uint8)    
# buf[0] = 1.1
# a = -2.8

# send_data = 0x200 >> 8
# print(send_data)
# send_data = 0x200 & 0xFF 
# print(send_data)

ports_list = list(serial.tools.list_ports.comports())  
plist = list(serial.tools.list_ports.comports())
if len(plist) <= 0:
    print("The Serial port can't find!")
    print('program end')
    sys.exit(0) 
else:
    for port in plist:
        uart_2_str = str(port)
        print(uart_2_str)
        uart_name_list = uart_2_str.split()
        print(uart_name_list)





