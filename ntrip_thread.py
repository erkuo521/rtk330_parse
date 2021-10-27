import os
import time
import threading
import json 
import struct
from logger import Logger
from ntrip_client import NtripClient
from ethernet_connect import EtherPort

class RuNtrip:
    def __init__(self, log_path):
        path = os.getcwd()
        self.local_time = time.localtime()
        self.formatted_file_time = time.strftime("%Y_%m_%d_%H_%M_%S", self.local_time)
        self.ether = EtherPort()

        # load json
        with open(path+'/setting/setting.json') as json_data:
            self.properties = json.load(json_data)

        # log path init
        self.dirpath = log_path
        self.log = Logger(self.dirpath, 'testlog')

        # ntrip init
        self.ntrip = NtripClient(self.properties, self.log, self.ntrip_receive_callback)
        self.ntrip_rtcm_logf = None 

    def ethernet_build_packet(self, dst, src, message_type, message_bytes=[]):
        packet = []
        packet.extend(message_type)
        msg_len = len(message_bytes)

        packet_len = struct.pack("<I", msg_len)

        packet.extend(packet_len)
        packet.extend(message_bytes)
        final_packet = packet  # + message_bytes

        msg_len = 2 + len(final_packet) + 2
        payload_len = struct.pack('H', msg_len)

        whole_packet = []
        header = dst + src + payload_len
        whole_packet.extend(header)

        whole_packet.extend([0x55, 0x55])
        whole_packet.extend(final_packet)
        whole_packet.extend(self.calc_crc(final_packet))
        if msg_len < 46:
            fill_bytes = bytes(46-msg_len)
            whole_packet.extend(fill_bytes)
        return bytes(whole_packet)

    '''
    ntrip
    '''

    def ntrip_client_thread(self):
        self.ntrip_rtcm_logf = open(os.path.join(self.dirpath, 'ntrip_rtcm_{0}.bin'.format(self.formatted_file_time)), "wb")

        self.ntrip.run()

    def ntrip_receive_callback(self, data):
        if data is not None:
            base_rtcm_packet = self.ethernet_build_packet(
                self.ether.get_dst_mac(),
                self.ether.get_src_mac(),
                b'\x02\x0b', bytes(data))
            self.ether.write(base_rtcm_packet)

            if self.ntrip_rtcm_logf is not None:
                self.ntrip_rtcm_logf.write(bytes(data))

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
        crc_msb = (crc & 0xFF00) >> 8
        crc_lsb = (crc & 0x00FF)
        return [crc_msb, crc_lsb]
    

