import minimalmodbus
import serial
import time
from itertools import repeat
import json

print("Start modbus")

class Jetsonmodbus:
	def __init__(self, Serial):
		self.errors = 0
        self.freq = 20 #hz
		self.mb = minimalmodbus.Instrument(Serial,1,minimalmodbus.MODE_RTU)
		self.mb.serial.baudrate = 115200         # Baud
		self.mb.serial.bytesize = 8
		self.mb.serial.parity   = serial.PARITY_NONE
		self.mb.serial.stopbits = 1
		self.mb.serial.timeout  = 1          # second
		self.mb.address = 1
		self.mb.clear_buffers_before_each_transaction = True
		self.holdRegisters = list(repeat(0, 29))
		self.inpRegisters = list(repeat(0, 29))
		self.open_cap = 0;
	
    def set_cap(self, open_cap)
        self.open_cap = open_cap
            

	def loop(self):
		self.life = True
		while self.life:
			self.registers = self.mb.read_registers(0, 30, 4)
			try:
				time.sleep(0.005)
                write_registers(0, self.holdRegisters)			
			except:
				self.errors = self.errors + 1
                
			time.sleep(1/self.freq)

		

"""
if __name__ == "__main__":
	jetboard = jetsonmodbus("/dev/ttyUSB1")
	jetboard.loop()
"""