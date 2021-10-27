from scapy.all import sendp, sniff, conf
import threading
import collections

class EtherPort:
    def __init__(self, options=None):
        self.iface = None

        self.src_mac = '38:14:28:3f:12:E2'
        self.dst_mac = 'ff:ff:ff:ff:ff:ff' 
        
        self.receive_cache = collections.deque(maxlen=1024*16)

    def get_network_card(self):
        network_card_info = []
        for item in conf.ifaces:
            if conf.ifaces[item].ip == '127.0.0.1' or conf.ifaces[item].mac == '':
                continue
            network_card_info.append(
                (conf.ifaces[item].name, conf.ifaces[item].mac))
        return network_card_info

    def get_src_mac(self):
        return bytes([int(x, 16) for x in self.src_mac.split(':')])

    def get_dst_mac(self):
        return bytes([int(x, 16) for x in self.dst_mac.split(':')])
    
    def write(self, data):  
        try:
            sendp(data, iface=self.iface, verbose=0)
            # print(data)
        except Exception as e:
            raise

