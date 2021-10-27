'''
For real time parsing to RTK330LA(fw24.1.24, related to rtk330la-UM20211018 version )
'''

import os
import time
import sys
import math
import serial
import serial.tools.list_ports
import struct
import datetime
import threading
import multiprocessing
from rtcm import rtcm_data as RTCM
from ntrip_thread import RuNtrip


preamble = bytearray.fromhex('5555')
packet_def = {
              's1': [43, bytearray.fromhex('7331')],\
              's2': [43, bytearray.fromhex('7332')],\
              'iN': [45, bytearray.fromhex('694E')],\
              'd1': [37, bytearray.fromhex('6431')],\
              'd2': [31, bytearray.fromhex('6432')],\
              'gN': [53, bytearray.fromhex('674E')],\
              'sT': [38, bytearray.fromhex('7354')],\
              'sP': [73, bytearray.fromhex('7350')]
              }

class rtk330la:
    def __init__(self, port, log_path, baud=115200, pipe=None):
        self.local_time = time.localtime()
        self.formatted_file_time = time.strftime("%Y_%m_%d_%H_%M_%S", self.local_time)
        self.port = port
        self.baud = baud
        self.dirpath = log_path
        self.physical_port = True
        if baud > 0:
            self.ser = serial.Serial(self.port, self.baud)
            self.open = self.ser.isOpen()
        else:
            self.ser = open('user_2021_08_20_02_46_30.bin', 'rb')
            self.open = True
            self.physical_port = False
            self.file_size = os.path.getsize('user_2021_08_20_02_46_30.bin')
        self.latest = []
        self.ready = False
        self.pipe = pipe
        self.size = 0
        self.header = None
        self.parser = None
        self.nmea = True 
        self.extra_port = True
        self.gga_str = ''

    def start(self, reset=False, reset_cmd='5555725300FC88'):
        if self.open:
            if self.physical_port:
                if reset is True:
                    self.ser.write(bytearray.fromhex(reset_cmd))
                self.ser.reset_input_buffer()
            while True:
                if self.physical_port:
                    read_size = self.ser.in_waiting
                else:
                    read_size = self.file_size
                data = self.ser.read(read_size)
                # print(data)
                if not data:
                    # end processing if reaching the end of the data file
                    if not self.physical_port:
                        break
                else:
                    self.nmea_logf(data)
                    self.parse_new_data(data)         
            # close port or file
            self.ser.close()
            print('End of processing.')
            self.pipe.send('exit')

    def nmea_logf(self, data):
        while self.nmea is True:
            data_str = str(data)
            self.char = '$'
            self.nPos = data_str.find(self.char)
            self.nmea_data = data_str[self.nPos:]
            if self.nmea_data.startswith('$GPGGA'):
                gga_data = data_str[self.nPos:(self.lens-5)]
                print(gga_data) 
                # record data to txt
                f_nmea = open(os.path.join(self.dirpath, 'data/nmea_data_{0}.txt'.format(self.formatted_file_time)), "a")
                f_nmea.write(str(gga_data) + '\n')
            else:
                break

    def parse_new_data(self, data):
        self.timestrt_hms = datetime.datetime.now().strftime('%H:%M:%S.%f') # hour,minute,second,to the millisecond
        self.timestrt_ymd = time.strftime('%Y-%m-%d', time.localtime()) # years,month,day
        self.lens = len(data)
        self.header = '0x55'
        self.char = '$'
        # Iterate over all the data
        for idx, i in enumerate(data):
            self.pos = idx
            self.header_1 = hex(i)
            # get nmea data
            # while self.nmea_logf is True:
            #     data_str = str(data)
            #     self.nPos = data_str.find(self.char)
            #     self.nmea_data = data_str[self.nPos:]
            #     if self.nmea_data.startswith('$GPGGA'):
            #         gga_data = data_str[self.nPos:(self.lens-5)]
            #         print(gga_data) 
            #         # record data to txt
            #         f_nmea = open('data/nmea_data_' + str(self.timestrt_ymd) + '.txt', 'a')
            #         f_nmea.write(str(gga_data) + '\n')
            #     else:
            #         break
            # match the packet type in data
            if self.header_1 == self.header and (self.pos+4) <= self.lens: 
                header_2 = hex(data[self.pos+1])  
                if header_2 == self.header and (self.pos+4) <= self.lens:
                    check_pos = hex(data[self.pos+2])
                    if check_pos == self.header:
                        continue
                    else:
                        packet_type = chr(data[self.pos+2]) + chr(data[self.pos+3])
                        if packet_type in packet_def.keys() and (self.pos+self.size) <= self.lens: 
                            self.size = packet_def[packet_type][0]
                            self.parser = eval('self.parse_' + packet_type)
                            # crc
                            self.packet_len = (self.pos+self.size)
                            if self.packet_len <= self.lens:
                                self.crc_f = data[self.pos+self.size-2]
                                self.crc_s = data[self.pos+self.size-1]
                                packet_crc = 256 * self.crc_f+ self.crc_s
                                calculated_crc = self.calc_crc(data[(self.pos+2):(self.pos+self.size-2)])
                                # decode
                                if packet_crc == calculated_crc:
                                    self.latest = self.parse_packet(data[(self.pos+2):(self.pos+self.size-2)])
                                    print(self.latest)
                                    # record packet data to txt 
                                    f_packet = open(
                                        os.path.join(self.dirpath, 'packet_' + packet_type + \
                                            '_{0}.txt'.format(self.formatted_file_time)), "a")
                                    f_packet.write(str(self.latest) + '\n')
                                else:
                                    print('crc fail')  

    def parse_packet(self, payload):
        data = self.parser(payload[3::])
        return data
    
    def parse_s1(self, payload):
        fmt = '<Idffffff'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        accels = data[2:5]
        angles = data[5:8]

        return gps_week, time_of_week, accels, angles

    def parse_s2(self, payload):
        fmt = '<Idffffff'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        accels = data[2:5]
        angles = data[5:8]

        return gps_week, time_of_week, accels, angles

    def parse_iN(self, payload):
        fmt = '<IdBBIIfhhhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        ins_status = data[2]
        ins_pos_status = data[3]
        latitude = data[4]
        longitude = data[5]
        hight = data[6]
        velocity_north = data[7]
        velocity_east = data[8]
        velocity_up = data[9]
        roll = data[10]
        pitch = data[11]
        head = data[12]

        return gps_week, time_of_week, ins_status, ins_pos_status, latitude, longitude, hight, velocity_north, \
            velocity_east, velocity_up, roll, pitch, head

    def parse_d1(self, payload):
        fmt = '=Idhhhhhhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        latitude = data[2]
        longitude = data[3]
        hight = data[4]
        velocity_north = data[5]
        velocity_east = data[6]
        velocity_up = data[7]
        roll = data[8]
        pitch = data[9]
        head = data[10]

        return gps_week, time_of_week, latitude, longitude, hight, velocity_north, velocity_east, velocity_up, \
            roll, pitch, head

    def parse_d2(self, payload):
        fmt = '<Idhhhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        latitude = data[2]
        longitude = data[3]
        hight = data[4]
        velocity_north = data[5]
        velocity_east = data[6]
        velocity_up = data[7]

        return gps_week, time_of_week, latitude, longitude, hight, velocity_north, velocity_east, velocity_up

    def parse_gN(self, payload):
        fmt = '=IdBIIfBfffhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        pos_mode = data[2]
        latitude = data[3]
        longitude = data[4]
        hight = data[5]
        num_of_SVs = data[6]
        hdop = data[7]
        vdop = data[8]
        tdop = data[9]
        diffage = data[10]
        velocity_north = data[11]
        velocity_east = data[12]
        velocity_up = data[13]

        return gps_week, time_of_week, pos_mode, latitude, longitude, hight, num_of_SVs, hdop, vdop, tdop, \
            diffage, velocity_north, velocity_east, velocity_up

    def parse_sT(self, payload):
        fmt = '<IdhBBBBBIff'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        year = data[2]
        month = data[3]
        day = data[4]
        hour = data[5]
        minute = data[6]
        sec = data[7]
        imu_status = data[8]
        imu_temp = data[9]
        mcu_temp = data[10]

        T = time_of_week
        num_week = int((T-18) / 86400)
        H = (int((T-18) / 3600) % 24) + 8
        M = int((T-18) / 60) % 60
        S = (T-18) - (int((T-18) / 60) * 60)
        time_stamp = (str(num_week) + "-" + str(H) + "-" + str(M) + "-" + str(S))

        return gps_week, time_of_week, year, month, day, hour, minute, sec, imu_status, \
             imu_temp, mcu_temp, time_stamp

    def parse_sP(self, payload):
        fmt = '<hIBBBBBBBfBdddddd'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        gps_ms = data[1]
        fix_status = data[2]
        data_status = data[3]
        nsat_use = data[4]
        nsat_view = data[5]
        hdop = data[6]
        ydop = data[7]
        pdop = data[8]
        geo_sep = data[9]
        leap_sec = data[10]
        latitude = data[11]
        longitude = data[12]
        height = data[13]
        north_vel = data[14]
        east_vel = data[15]
        up_vel = data[16]

        return gps_week, gps_ms, fix_status, data_status, nsat_use, nsat_view, hdop, ydop, \
            pdop, geo_sep, leap_sec, latitude, longitude, height, north_vel, east_vel, up_vel


    def calc_crc(self, payload):
        crc = 0x1D0F
        for bytedata in payload:
            crc = crc^(bytedata << 8) 
            for i in range(0,8):
                if crc & 0x8000:
                    crc = (crc << 1)^0x1021
                else:
                    crc = crc << 1

        crc = crc & 0xffff
        return crc

if __name__ == "__main__":
    port = 'COM55'
    baud = 115200
    path = os.getcwd()
    if  not os.path.exists(path+'/data/'):
        os.makedirs(path+'/data/')
    log_path = path + '/data/log_rtk330la_' + time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    os.makedirs(log_path)

    num_of_args = len(sys.argv)
    if num_of_args > 1:
        port = sys.argv[1]
        if num_of_args > 2:
            baud = int(sys.argv[2])
            if num_of_args > 3:
                packet_type = sys.argv[3]

   
    unit_parse = rtk330la(port, log_path, baud, pipe=None)
    unit_rtcm = RTCM(log_path)
    unit_ntrip = RuNtrip(log_path).ntrip_client_thread

    parse_thread = threading.Thread(target=unit_parse.start)
    rtcm_thread = threading.Thread(target=unit_rtcm.rtcm_start)
    ntrip_thread = threading.Thread(target=unit_ntrip)
    threads = []
    threads.append(parse_thread)
    threads.append(rtcm_thread)
    threads.append(ntrip_thread)
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    
    

  