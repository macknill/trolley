import modbus
import threading
import os
import time
import subprocess
import datetime
import time

stmboard = modbus.Jetsonmodbus("/dev/ttyUSB0")

thread_modbus = threading.Thread(target = stmboard.loop, name = "proc_jetboard")
thread_modbus.start()

print("start car drive!!!")
try:
    while True:
        print (stmboard.read_reg(2))
        time.sleep(0.5)        
        print(stmboard.inpRegisters)
        stmboard.write_reg(3, 160)
        time.sleep(0.5)        
         

except KeyboardInterrupt:
    stmboard.life = 0
    print("end code")
