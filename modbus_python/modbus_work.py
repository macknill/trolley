import modbus_loop
import threading
import os
import time
import subprocess
import datetime
import time


stmboard = modbus_loop.Jetsonmodbus("/dev/ttyUSB0")

thread_modbus = threading.Thread(target = stmboard.loop, name = "proc_jetboard")
thread_modbus.start()

print("start car drive!!!")

inc = 0;

try:
    while True:
        print ("////////////")
        print ("write: " + str(inc))
        stmboard.write_reg(0, inc, 0)
        print ("Reg: " + str(stmboard.read_reg(0)))
        time.sleep(0.75)
        if inc < 4:
            inc += 1
        else:
            inc = 0

except KeyboardInterrupt:
    stmboard.life = 0
    print("end code")
