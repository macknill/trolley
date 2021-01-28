import minimalmodbus
import serial
import time
from itertools import repeat


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
        self.registers_quantity = 30
        self.holdRegisters = list(repeat(0, self.registers_quantity))
        self.inpRegisters = list(repeat(0, self.registers_quantity))
        self.write_cmd = 0;

    def write_reg(self, reg_addr, reg_data):
        if reg_addr < self.registers_quantity:
            self.holdRegisters[reg_addr] = int(reg_data)
            self.write_cmd = 1;
    def read_reg(self, reg_addr):
        if reg_addr < self.registers_quantity:
            return self.inpRegisters[reg_addr]

    def loop(self):
        self.life = True
        while self.life:
            self.inpRegisters = self.mb.read_registers(0, self.registers_quantity, 3)
            if self.write_cmd == 1:
                self.write_cmd == 0
                try:
                    time.sleep(0.01)
                    write_registers(0, self.holdRegisters)			
                except:
                    self.errors = self.errors + 1
                    
            time.sleep(1/self.freq)
