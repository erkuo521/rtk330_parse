import serial 
import time 
import os
import threading

class rtcm_data:
    def __init__(self, log_path, *args):
        self.gnss_port = 'COM56'
        self.debug_port = 'COM57'
        self.gnss_baud = 460800
        self.debug_baud = 460800
        self.gnss_physical_port = True
        self.debug_physical_port = True
        self.dirpath = log_path

        self.gnss_ser = serial.Serial(self.gnss_port, self.gnss_baud)
        self.gp_open = self.gnss_ser.isOpen

        self.debug_ser = serial.Serial(self.debug_port, self.debug_baud)
        self.dp_open = self.debug_ser.isOpen


    def rtcm_start(self):
        if self.gp_open and self.dp_open:
            while True:
                thread1 = threading.Thread(target=self.gnss_data)
                thread2 = threading.Thread(target=self.debug_data)
                threads = []
                threads.append(thread1)
                threads.append(thread2)
                for t in threads:
                    t.start()   
                for t in threads:
                    t.join()      

    def gnss_data(self):
        formatted_file_time = time.strftime("%Y_%m_%d_%H_%M_%S")
        while self.gnss_physical_port is True:
            gnss_count = self.gnss_ser.inWaiting
            if gnss_count !=0 :
                gnss_recv = self.gnss_ser.read(self.gnss_ser.in_waiting) 
                gnss_logf = open(
                    os.path.join(self.dirpath, 'rtcm_rover_{0}.bin'.format(formatted_file_time)), "ab")
                gnss_logf.write(gnss_recv)

    def debug_data(self):
        formatted_file_time = time.strftime("%Y_%m_%d_%H_%M_%S")
        while self.debug_physical_port is True:
            debug_count = self.debug_ser.inWaiting
            if debug_count !=0 :
                debug_recv = self.debug_ser.read(self.debug_ser.in_waiting)
                debug_logf = open(
                    os.path.join(self.dirpath, 'rtcm_base_{0}.bin'.format(formatted_file_time)), "ab")
                debug_logf.write(debug_recv)
    
