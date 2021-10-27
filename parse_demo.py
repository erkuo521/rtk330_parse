import os
import sys
import argparse
import time
import datetime
import collections
import struct
import math
import operator

is_windows = sys.platform.__contains__(
    'win32') or sys.platform.__contains__('win64')
is_later_py_38 = sys.version_info > (3, 8)
is_later_py_3 = sys.version_info > (3, 0)

class SerialParse:
    def __init__(self, file_path, path, inskml_rate):

        self.f_bin = open(file_path, 'rb')
        self.bin_zise = os.path.getsize(file_path)
        self.path = path

        self.inskml_rate = 1/inskml_rate

        self.state = 0
        self.frame = []
        self.sync_pattern = collections.deque(2*[0], 2)
        self.payload_type = 0
        self.payload_len = 0

        self.nmea_buffer = []
        self.nmea_sync = 0

        self.f_nmea = None
        self.f_process = None
        self.f_imu = None
        self.f_gnssposvel = None
        self.f_ins = None
        self.f_odo = None
        self.f_gnss_kml = None
        self.f_ins_kml = None

        self.gnssdata = []
        self.insdata = []
        self.gNdata = []

    def start_pasre(self):
        self.f_s1 = open(self.path + 's1.csv', 'w')
        self.f_s2 = open(self.path + 's2.csv', 'w')
        self.f_gN = open(self.path + 'gN.csv', 'w')
        self.f_iN = open(self.path + 'iN.csv', 'w')
        self.f_d1 = open(self.path + 'd1.csv', 'w')
        self.f_d2 = open(self.path + 'd2.csv', 'w')
        self.f_sT = open(self.path + 'sT.csv', 'w')
        self.f_o1 = open(self.path + 'o1.csv', 'w')

        self.f_process = open(self.path[0:-1] + '-process', 'w')
        self.f_gnssposvel = open(self.path[0:-1] + '-gnssposvel.txt', 'w')
        self.f_imu = open(self.path[0:-1] + '-imu.txt', 'w')
        self.f_ins = open(self.path[0:-1] + '-ins.txt', 'w')
        self.f_nmea = open(self.path[0:-1] + '-nmea', 'wb')
        self.f_odo = open(self.path[0:-1] + '-odo.txt', 'w')

        self.f_gnss_kml = open(self.path[0:-1] + '-gnss.kml', 'w')
        self.f_ins_kml = open(self.path[0:-1] + '-ins.kml', 'w')

        readlen = 0
        readsize = 1024
        while readlen < self.bin_zise:
            array_buf = self.f_bin.read(readsize)
            for i, new_byte in enumerate(array_buf):
                self.parse_uart_packet(new_byte)
                self.parse_nmea(new_byte)
            readlen = readlen + readsize

        self.save_gnss_kml()
        self.save_ins_kml()
        
        self.f_gnss_kml.close()
        self.f_ins_kml.close()

        self.f_odo.close()
        self.f_nmea.close()
        self.f_imu.close()
        self.f_ins.close()
        self.f_process.close()
        self.f_gnssposvel.close()

        self.f_s1.close()
        self.f_s2.close()
        self.f_gN.close()
        self.f_iN.close()
        self.f_d1.close()
        self.f_d2.close()
        self.f_sT.close()
        self.f_o1.close()

        self.f_bin.close()

    def parse_uart_packet(self, data_block):
        '''
            0 : # check the sync header
            1 : # check the packet type
            2 : # recv the packet length
            3 : # recv packet payload, check the crc16, check the sync end
        '''
        if self.state == 0:
            self.sync_pattern.append(data_block)
        else:
            self.frame.append(data_block)

        if self.state == 0:
            if operator.eq(list(self.sync_pattern), [0x55, 0x55]):
                self.frame = [0x55, 0x55]
                self.state = 1

        elif self.state == 1:
            if len(self.frame) == 4:
                # check the packet type or not
                self.payload_type = self.frame[2] + (self.frame[3] << 8)
                self.state = 2

        elif self.state == 2:
            self.payload_len = self.frame[4]
            self.state = 3

        elif self.state == 3:
            if len(self.frame) == 5 + self.payload_len + 2:
                crc = calc_crc(self.frame[2:-2])
                if crc == (self.frame[-2] << 8) + self.frame[-1]:
                    # find a whole frame
                    self.uart_rtx_packet(self.payload_type, self.payload_len, self.frame[5:-2])

                self.state = 0
                self.sync_pattern = collections.deque(2*[0], 2)

        else:
            self.state = 0

    def uart_rtx_packet(self, type, length, content):
        ''' s1  0x3173
            s2  0x3273
            gN  0x4E67
            iN  0x4E69
            d1  0x3164
            d2  0x3264
            sT  0x5473
            o1  0x316F
        '''
        b = struct.pack('{0}B'.format(length), *content)

        if type == 0x3173:
            data = struct.unpack('<Idffffff', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1], "11.4f") + ","
            buffer = buffer + format(data[2], "14.10f") + ","
            buffer = buffer + format(data[3], "14.10f") + ","
            buffer = buffer + format(data[4], "14.10f") + ","
            buffer = buffer + format(data[5], "14.10f") + ","
            buffer = buffer + format(data[6], "14.10f") + ","
            buffer = buffer + format(data[7], "14.10f") + "\n"
            self.f_s1.write(buffer)

        elif type == 0x3273:
            data = struct.unpack('<Idffffff', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1], "11.4f") + ","
            buffer = buffer + format(data[2], "14.10f") + ","
            buffer = buffer + format(data[3], "14.10f") + ","
            buffer = buffer + format(data[4], "14.10f") + ","
            buffer = buffer + format(data[5], "14.10f") + ","
            buffer = buffer + format(data[6], "14.10f") + ","
            buffer = buffer + format(data[7], "14.10f") + "\n"
            self.f_s2.write(buffer)
				
            sbuffer = format(data[0], "") + ","
            sbuffer = sbuffer + format(data[1], "11.4f") + "," + "    ,"
            sbuffer = sbuffer + format(data[2], "14.10f") + ","
            sbuffer = sbuffer + format(data[3], "14.10f") + ","
            sbuffer = sbuffer + format(data[4], "14.10f") + ","
            sbuffer = sbuffer + format(data[5], "14.10f") + ","
            sbuffer = sbuffer + format(data[6], "14.10f") + ","
            sbuffer = sbuffer + format(data[7], "14.10f") + "\n"
            self.f_imu.write(sbuffer)
            self.f_process.write('$GPIMU,'+ sbuffer)

        elif type == 0x4E67:
            data = struct.unpack('<IdBiifBfffHhhh', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1], "11.4f") + ","
            buffer = buffer + format(data[2], "3") + ","
            buffer = buffer + format(data[3]*180/2147483648, "14.9f") + ","
            buffer = buffer + format(data[4]*180/2147483648, "14.9f") + ","
            buffer = buffer + format(data[5], "10.4f") + ","
            buffer = buffer + format(data[6], "3") + ","
            buffer = buffer + format(data[7], "5.1f") + ","
            buffer = buffer + format(data[8], "5.1f") + ","
            buffer = buffer + format(data[9], "5.1f") + ","
            buffer = buffer + format(data[10], "5.1f") + ","
            buffer = buffer + format(data[11]/100, "10.4f") + ","
            buffer = buffer + format(data[12]/100, "10.4f") + ","
            buffer = buffer + format(data[13]/100, "10.4f") + "\n"
            self.f_gN.write(buffer)

            self.gNdata = data
            self.gnssdata.append(data)

        elif type == 0x4E69:
            data = struct.unpack('<IdBBiifhhhhhh', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1], "11.4f") + ","
            buffer = buffer + format(data[2], "3") + ","
            buffer = buffer + format(data[3], "3") + ","
            buffer = buffer + format(data[4]*180/2147483648, "14.9f") + ","
            buffer = buffer + format(data[5]*180/2147483648, "14.9f") + ","
            buffer = buffer + format(data[6], "10.4f") + ","
            buffer = buffer + format(data[7]/100, "10.4f") + ","
            buffer = buffer + format(data[8]/100, "10.4f") + ","
            buffer = buffer + format(data[9]/100, "10.4f") + ","
            buffer = buffer + format(data[10]/100, "14.9f") + ","
            buffer = buffer + format(data[11]/100, "14.9f") + ","
            buffer = buffer + format(data[12]/100, "14.9f") + "\n"
            self.f_iN.write(buffer)

            if math.fmod(data[1]+0.0005, 0.1) <= 0.005:
                sbuffer = format(data[0], "") + ","
                sbuffer = sbuffer + format(data[1], "11.4f") + ","
                sbuffer = sbuffer + format(data[4]*180/2147483648, "14.9f") + ","
                sbuffer = sbuffer + format(data[5]*180/2147483648, "14.9f") + ","
                sbuffer = sbuffer + format(data[6], "10.4f") + ","
                sbuffer = sbuffer + format(data[7]/100, "10.4f") + ","
                sbuffer = sbuffer + format(data[8]/100, "10.4f") + ","
                sbuffer = sbuffer + format(data[9]/100, "10.4f") + ","
                sbuffer = sbuffer + format(data[10]/100, "14.9f") + ","
                sbuffer = sbuffer + format(data[11]/100, "14.9f") + ","
                sbuffer = sbuffer + format(data[12]/100, "14.9f") + ","
                sbuffer = sbuffer + format(data[3], "3")
                self.f_ins.write(sbuffer + "," + format(data[2], "3") + "\n")
                self.f_process.write('$GPINS,'+ sbuffer + "\n")

                if abs(data[5]*data[4]) > 0.00000001:
                    self.insdata.append(data)

        elif type == 0x3164:
            data = struct.unpack('<Idhhhhhhhhh', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1], "11.4f") + ","
            buffer = buffer + format(data[2]/100, "8.3f") + ","
            buffer = buffer + format(data[3]/100, "8.3f") + ","
            buffer = buffer + format(data[4]/100, "8.3f") + ","
            buffer = buffer + format(data[5]/100, "8.3f") + ","
            buffer = buffer + format(data[6]/100, "8.3f") + ","
            buffer = buffer + format(data[7]/100, "8.3f") + ","
            buffer = buffer + format(data[8]/100, "8.3f") + ","
            buffer = buffer + format(data[9]/100, "8.3f") + ","
            buffer = buffer + format(data[10]/100, "8.3f") + "\n"
            self.f_d1.write(buffer)

        elif type == 0x3264:
            data = struct.unpack('<Idhhhhhh', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1], "11.4f") + ","
            buffer = buffer + format(data[2]/100, "8.3f") + ","
            buffer = buffer + format(data[3]/100, "8.3f") + ","
            buffer = buffer + format(data[4]/100, "8.3f") + ","
            buffer = buffer + format(data[5]/100, "8.3f") + ","
            buffer = buffer + format(data[6]/100, "8.3f") + ","
            buffer = buffer + format(data[7]/100, "8.3f") + "\n"
            self.f_d2.write(buffer)
			
            if self.gNdata != None and self.gNdata != []:
                sbuffer = '$GPGNSS,'
                sbuffer = sbuffer + format(self.gNdata[0], "") + ","
                sbuffer = sbuffer + format(self.gNdata[1], "11.4f") + ","
                sbuffer = sbuffer + format(self.gNdata[3]*180/2147483648, "14.9f") + ","
                sbuffer = sbuffer + format(self.gNdata[4]*180/2147483648, "14.9f") + ","
                sbuffer = sbuffer + format(self.gNdata[5], "10.4f") + ","
                sbuffer = sbuffer + format(data[2]/100, "8.3f") + ","
                sbuffer = sbuffer + format(data[3]/100, "8.3f") + ","
                sbuffer = sbuffer + format(data[4]/100, "8.3f") + ","
                sbuffer = sbuffer + format(self.gNdata[2], "3") + "\n"
                self.f_process.write(sbuffer)

                sbuffer = '$GPVEL,'
                sbuffer = sbuffer + format(self.gNdata[0], "") + ","
                sbuffer = sbuffer + format(self.gNdata[1], "11.4f") + ","
                north_vel = self.gNdata[11]/100
                east_vel = self.gNdata[12]/100
                up_vel = self.gNdata[13]/100
                horizontal_speed = math.sqrt(
					north_vel * north_vel + east_vel * east_vel)
                track_over_ground = math.atan2(
					east_vel, north_vel) * (57.295779513082320)
                sbuffer = sbuffer + format(horizontal_speed, "10.4f") + ","
                sbuffer = sbuffer + format(track_over_ground, "10.4f") + ","
                sbuffer = sbuffer + format(up_vel, "10.4f") + "\n"
                self.f_process.write(sbuffer)

                e_buffer = format(self.gNdata[0], "") + ","
                e_buffer = e_buffer + format(self.gNdata[1], "11.4f") + ","
                e_buffer = e_buffer + format(self.gNdata[3]*180/2147483648, "14.9f") + ","
                e_buffer = e_buffer + format(self.gNdata[4]*180/2147483648, "14.9f") + ","
                e_buffer = e_buffer + format(self.gNdata[5], "10.4f") + ","
                e_buffer = e_buffer + format(data[2]/100, "8.3f") + ","
                e_buffer = e_buffer + format(data[3]/100, "8.3f") + ","
                e_buffer = e_buffer + format(data[4]/100, "8.3f") + ","
                e_buffer = e_buffer + format(self.gNdata[2], "3") + ","
                e_buffer = e_buffer + format(north_vel, "10.4f") + ","
                e_buffer = e_buffer + format(east_vel, "10.4f") + ","
                e_buffer = e_buffer + format(up_vel, "10.4f") + ","
                e_buffer = e_buffer + format(track_over_ground, "10.4f") + "\n"
                self.f_gnssposvel.write(e_buffer)

        elif type == 0x5473:
            data = struct.unpack('<IdHBBBBBIff', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1], "11.4f") + ","
            buffer = buffer + format(data[2], "5") + ","
            buffer = buffer + format(data[3], "5") + ","
            buffer = buffer + format(data[4], "5") + ","
            buffer = buffer + format(data[5], "5") + ","
            buffer = buffer + format(data[6], "5") + ","
            buffer = buffer + format(data[7], "5") + ","
            buffer = buffer + format(data[8], "5") + ","
            buffer = buffer + format(data[9], "8.3") + ","
            buffer = buffer + format(data[10], "8.3") + "\n"
            self.f_sT.write(buffer)

        elif type == 0x316F:
            data = struct.unpack('<HIBdBQ', b)

            buffer = format(data[0], "") + ","
            buffer = buffer + format(data[1]/1000, "11.4f") + ","
            buffer = buffer + format(data[2], "3") + ","
            buffer = buffer + format(data[3], "10.4f") + ","
            buffer = buffer + format(data[4], "3") + ","
            buffer = buffer + format(data[5], "16") + "\n"
            self.f_o1.write(buffer)
            self.f_odo.write(buffer)
            self.f_process.write('$GPODO,' + buffer)

    def parse_nmea(self, bytedata):
        if bytedata == 0x24:
            self.nmea_sync = 1
            self.nmea_buffer = []
            self.nmea_buffer.append(bytedata)
        else:
            self.nmea_buffer.append(bytedata)
            if self.nmea_sync == 1:
                if bytedata == 0x0D:
                    self.nmea_sync = 2
            elif self.nmea_sync == 2:
                if bytedata == 0x0A:
                    if len(self.nmea_buffer) > 10 and self.nmea_buffer[-5] == 0x2A:
                        check = nmea_checksum(self.nmea_buffer[1:-2])
                        if check == True:
                            self.f_nmea.write(bytes(self.nmea_buffer))
                self.nmea_buffer = []
                self.nmea_sync = 0

    def weeksecondstoutc(self, gpsweek, gpsseconds, leapseconds):
        datetimeformat = "%Y-%m-%d %H:%M:%S"
        epoch = datetime.datetime.strptime(
            "1980-01-06 00:00:00", datetimeformat)
        elapsed = datetime.timedelta(
            days=(gpsweek*7), seconds=(gpsseconds+leapseconds))
        return datetime.datetime.strftime(epoch + elapsed, datetimeformat)

    def save_gnss_kml(self):
        # white-cyan, red, purple, light-yellow, green, yellow
        color = ["ffffffff", "ff0000ff", "ffff00ff",
                 "50FF78F0", "ff00ff00", "ff00aaff"]
        kml_header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"\
            + "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"\
            + "<Document>\n"
        for i in range(6):
            kml_header += "<Style id=\"P" + str(i) + "\">\r\n"\
                + "<IconStyle>\r\n"\
                + "<color>" + color[i] + "</color>\n"\
                + "<scale>0.3</scale>\n"\
                + "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/track.png</href></Icon>\n"\
                + "</IconStyle>\n"\
                + "</Style>\n"
        self.f_gnss_kml.write(kml_header)

        gnss_postype = ["NONE", "PSRSP", "PSRDIFF",
                        "UNDEFINED", "RTKFIXED", "RTKFLOAT"]

        gnss_track = "<Placemark>\n"\
            + "<name>Rover Track</name>\n"\
            + "<Style>\n"\
            + "<LineStyle>\n"\
            + "<color>ffffffff</color>\n"\
            + "</LineStyle>\n"\
            + "</Style>\n"\
            + "<LineString>\n"\
            + "<coordinates>\n"

        for pos in self.gnssdata:
            if pos[2] == 0:
                continue

            gnss_track += format(pos[4]*180/2147483648, ".9f") + ',' + format(
                pos[3]*180/2147483648, ".9f") + ',' + format(pos[5], ".3f") + '\n'

        gnss_track += "</coordinates>\n"\
            + "</LineString>\n"\
            + "</Placemark>\n"

        gnss_track += "<Folder>\n"\
            + "<name>Rover Position</name>\n"

        for i, pos in enumerate(self.gnssdata):
            ep = self.weeksecondstoutc(pos[0], pos[1]*1000/1000, -18)
            ep_sp = time.strptime(ep, "%Y-%m-%d %H:%M:%S")

            if pos[2] == 0:
                pass
            else:
                track_ground = math.atan2(
                    pos[12]/100, pos[11]/100) * (57.295779513082320)

                gnss_track += "<Placemark>\n"
                if i <= 1:
                    gnss_track += "<name>Start</name>\n"
                elif i == len(self.gnssdata)-1:
                    gnss_track += "<name>End</name>\n"
                else:
                    if math.fmod(ep_sp[5]+((pos[1]*1000) % 1000)/1000+0.025, 30) < 0.05:
                        gnss_track += "<name>"\
                            + "%02d" % ep_sp[3] + "%02d" % ep_sp[4] + "%02d" % ep_sp[5]\
                            + "</name>\n"

                gnss_track += "<TimeStamp><when>"\
                    + time.strftime("%Y-%m-%dT%H:%M:%S.", ep_sp)\
                    + "%02dZ" % (((pos[1]*1000) % 1000)/10)\
                    + "</when></TimeStamp>\n"

                gnss_track += "<description><![CDATA[\n"\
                    + "<TABLE border=\"1\" width=\"100%\" Align=\"center\">\n"\
                    + "<TR ALIGN=RIGHT>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>"\
                    + str(pos[0]) + "</TD><TD>" + "%.3f" % ((pos[1]*1000)/1000) + "</TD><TD>"\
                    + "%2d:%2d:%7.4f" % (ep_sp[3], ep_sp[4], ep_sp[5]+((pos[1]*1000) % 1000)/1000) + "</TD><TD>"\
                    + "%4d/%2d/%2d" % (ep_sp[0], ep_sp[1], ep_sp[2]) + "</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>"\
                    + "%.8f" % (pos[3]*180/2147483648) + "</TD><TD>" + "%.8f" % (pos[4]*180/2147483648) + "</TD><TD>" + "%.4f" % pos[5] + "</TD><TD>(DMS,m)</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>"\
                    + "%.4f" % (pos[11]/100) + "</TD><TD>" + "%.4f" % (pos[12]/100) + "</TD><TD>" + "%.4f" % (-pos[13]/100) + "</TD><TD>(m/s)</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>"\
                    + "0" + "</TD><TD>" + "0" + "</TD><TD>" + "%.4f" % track_ground + "</TD><TD>(deg,approx)</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>"\
                    + "0" + "</TD><TD>" + gnss_postype[pos[2]] + "</TD><TR>\n"\
                    + "</TABLE>\n"\
                    + "]]></description>\n"

                gnss_track += "<styleUrl>#P" + str(pos[2]) + "</styleUrl>\n"\
                    + "<Style>\n"\
                    + "<IconStyle>\n"\
                    + "<heading>" + "%.4f" % track_ground + "</heading>\n"\
                    + "</IconStyle>\n"\
                    + "</Style>\n"

                gnss_track += "<Point>\n"\
                    + "<coordinates>" + "%.9f,%.9f,%.3f" % (pos[4]*180/2147483648, pos[3]*180/2147483648, pos[5]) + "</coordinates>\n"\
                    + "</Point>\n"

                gnss_track += "</Placemark>\n"

        gnss_track += "</Folder>\n"\
            + "</Document>\n"\
            + "</kml>\n"

        self.f_gnss_kml.write(gnss_track)

    def save_ins_kml(self):
        '''
        '''
        # white-cyan, red, purple, light-yellow, green, yellow
        color = ["ffffffff", "50FF78F0", "ffff00ff",
                 "ff0000ff", "ff00ff00", "ff00aaff"]
        kml_header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"\
            + "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"\
            + "<Document>\n"
        for i in range(6):
            kml_header += "<Style id=\"P" + str(i) + "\">\r\n"\
                + "<IconStyle>\r\n"\
                + "<color>" + color[i] + "</color>\n"\
                + "<scale>0.3</scale>\n"\
                + "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/track.png</href></Icon>\n"\
                + "</IconStyle>\n"\
                + "</Style>\n"
        self.f_ins_kml.write(kml_header)

        ins_track = "<Placemark>\n"\
            + "<name>Rover Track</name>\n"\
            + "<Style>\n"\
            + "<LineStyle>\n"\
            + "<color>ff0000ff</color>\n"\
            + "</LineStyle>\n"\
            + "</Style>\n"\
            + "<LineString>\n"\
            + "<coordinates>\n"

        ins_status = ["INS_INACTIVE", "INS_ALIGNING", "INS_HIGH_VARIANCE",
                      "INS_SOLUTION_GOOD", "INS_SOLUTION_FREE", "INS_ALIGNMENT_COMPLETE"]
        ins_postype = ["INS_NONE", "INS_PSRSP", "INS_PSRDIFF",
                       "INS_PROPOGATED", "INS_RTKFIXED", "INS_RTKFLOAT"]

        for ins in self.insdata:
            ep = self.weeksecondstoutc(ins[0], ins[1]*1000/1000, -18)
            ep_sp = time.strptime(ep, "%Y-%m-%d %H:%M:%S")

            if math.fmod(ep_sp[5]+((ins[1]*1000) % 1000)/1000+0.0005, self.inskml_rate) < 0.005:
                if abs(ins[5]*ins[4]) < 0.00000001:
                    continue

                ins_track += format(ins[5]*180/2147483648, ".9f") + ',' + format(
                    ins[4]*180/2147483648, ".9f") + ',' + format(ins[6], ".3f") + '\n'

        ins_track += "</coordinates>\n"\
            + "</LineString>\n"\
            + "</Placemark>\n"

        ins_track += "<Folder>\n"\
            + "<name>Rover Position</name>\n"

        for i, ins in enumerate(self.insdata):
            ep = self.weeksecondstoutc(ins[0], ins[1]*1000/1000, -18)
            ep_sp = time.strptime(ep, "%Y-%m-%d %H:%M:%S")

            if i == 0 or i == len(self.insdata)-1 or math.fmod(ins[1]*1000/1000 + 0.0005, self.inskml_rate) < 0.005:
                ins_track += "<Placemark>\n"
                if i <= 1:
                    ins_track += "<name>Start</name>\n"
                elif i == len(self.insdata)-1:
                    ins_track += "<name>End</name>\n"
                else:
                    if math.fmod(ep_sp[5]+((ins[1]*1000) % 1000)/1000+0.025, 30) < 0.05:
                        ins_track += "<name>"\
                            + "%02d" % ep_sp[3] + "%02d" % ep_sp[4] + "%02d" % ep_sp[5]\
                            + "</name>\n"

                ins_track += "<TimeStamp><when>"\
                    + time.strftime("%Y-%m-%dT%H:%M:%S.", ep_sp)\
                    + "%02dZ" % (((ins[1]*1000) % 1000)/10)\
                    + "</when></TimeStamp>\n"

                ins_track += "<description><![CDATA[\n"\
                    + "<TABLE border=\"1\" width=\"100%\" Align=\"center\">\n"\
                    + "<TR ALIGN=RIGHT>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>"\
                    + str(ins[0]) + "</TD><TD>" + "%.3f" % (ins[1]*1000/1000) + "</TD><TD>"\
                    + "%2d:%2d:%7.4f" % (ep_sp[3], ep_sp[4], ep_sp[5]+((ins[1]*1000) % 1000)/1000) + "</TD><TD>"\
                    + "%4d/%2d/%2d" % (ep_sp[0], ep_sp[1], ep_sp[2]) + "</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>"\
                    + "%.8f" % (ins[4]*180/2147483648) + "</TD><TD>" + "%.8f" % (ins[5]*180/2147483648) + "</TD><TD>" + "%.4f" % ins[6] + "</TD><TD>(DMS,m)</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>"\
                    + "%.4f" % (ins[7]/100) + "</TD><TD>" + "%.4f" % (ins[8]/100) + "</TD><TD>" + "%.4f" % (-ins[9]/100) + "</TD><TD>(m/s)</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>"\
                    + "%.4f" % (ins[10]/100) + "</TD><TD>" + "%.4f" % (ins[11]/100) + "</TD><TD>" + "%.4f" % (ins[12]/100) + "</TD><TD>(deg,approx)</TD></TR>\n"\
                    + "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>"\
                    + ins_status[ins[2]] + "</TD><TD>" + ins_postype[ins[3]] + "</TD><TR>\n"\
                    + "</TABLE>\n"\
                    + "]]></description>\n"

                pcolor = 0
                if ins[3] == 0:     # "INS_INACTIVE"
                    pcolor = 0
                elif ins[3] == 1:   # "SPP/INS_SPP"
                    pcolor = 1
                elif ins[3] == 2:   # "PSRDIFF/INS_PSRDIFF (RTD)"
                    pcolor = 2
                elif ins[3] == 3:   # "INS_DR"
                    pcolor = 3
                elif ins[3] == 4:   # "RTK_FIX/INS_RTKFIXED"
                    pcolor = 4
                elif ins[3] == 5:   # "RTK_FLOAT/INS_RTKFLOAT"
                    pcolor = 5
                # pcolor = 4

                ins_track += "<styleUrl>#P" + str(pcolor) + "</styleUrl>\n"\
                    + "<Style>\n"\
                    + "<IconStyle>\n"\
                    + "<heading>" + "%.4f" % (ins[12]/100) + "</heading>\n"\
                    + "</IconStyle>\n"\
                    + "</Style>\n"

                ins_track += "<Point>\n"\
                    + "<coordinates>" + "%.9f,%.9f,%.3f" % (ins[5]*180/2147483648, ins[4]*180/2147483648, ins[6]) + "</coordinates>\n"\
                    + "</Point>\n"

                ins_track += "</Placemark>\n"

        ins_track += "</Folder>\n"\
            + "</Document>\n"\
            + "</kml>\n"

        self.f_ins_kml.write(ins_track)


class EtherParse:
    def __init__(self, file_path, path):

        self.f_bin = open(file_path, 'rb')
        self.bin_zise = os.path.getsize(file_path)
        self.path = path

        self.state = 0
        self.frame = []
        self.sync_pattern = collections.deque(2*[0], 2)
        self.payload_type = 0
        self.payload_len = 0

        self.nmea_buffer = []
        self.nmea_sync = 0

        self.f_nmea = None
        self.f_corrimu = None
        self.f_rover_rtcm = None


    def start_pasre(self):
        self.f_nmea = open(self.path[0:-1] + '-nmea', 'wb')
        self.f_corrimu = open(self.path[0:-1] + '-corrimu.txt', 'w')
        self.f_rover_rtcm = open(self.path[0:-1] + '-rover_rtcm', 'wb')

        readlen = 0
        readsize = 1024

        while readlen < self.bin_zise:
            array_buf = self.f_bin.read(readsize)
            for i, new_byte in enumerate(array_buf):
                self.parse_eth_packet(new_byte)
                self.parse_nmea(new_byte)
            readlen = readlen + readsize
        
        self.f_nmea.close()
        self.f_corrimu.close()
        self.f_rover_rtcm.close()
        self.f_bin.close()
        
    def parse_eth_packet(self, data_block):
        '''
            0 : # check the sync header
            1 : # check the packet type
            2 : # recv the packet length
            3 : # recv packet payload, check the crc16, check the sync end
        '''
        if self.state == 0:
            self.sync_pattern.append(data_block)
        else:
            self.frame.append(data_block)

        if self.state == 0:
            if operator.eq(list(self.sync_pattern), [0x55, 0x55]):
                self.frame = [0x55, 0x55]
                self.state = 1

        elif self.state == 1:
            if len(self.frame) == 4:
                # check the packet type or not
                self.payload_type = self.frame[2] + (self.frame[3] << 8)
                self.state = 2

        elif self.state == 2:
            if len(self.frame) == 8:
                self.payload_len = self.frame[4] + (self.frame[5] << 8) + (self.frame[6] << 16) + (self.frame[7] << 24)
                self.state = 3

        elif self.state == 3:
            if len(self.frame) == 8 + self.payload_len + 2:
                crc = calc_crc(self.frame[2:-2])
                if crc == (self.frame[-2] << 8) + self.frame[-1]:
                    # find a whole frame
                    self.eth_rtx_packet(self.payload_type, self.payload_len, self.frame[8:-2])

                self.state = 0
                self.sync_pattern = collections.deque(2*[0], 2)

        else:
            self.state = 0

    def eth_rtx_packet(self, type, length, content):
        ''' Parse final packet
        '''
        if type == 0x0a06: # rover rtcm
            self.f_rover_rtcm.write(bytes(content))

        elif type == 0x0a07: # corr imu
            b = struct.pack('{0}B'.format(length), *content)
            data = struct.unpack('<HIffffff', b)
            buffer = format(data[0], '') + ",  "
            buffer = buffer + format(data[1]/1000, '11.4f') + ","
            buffer = buffer + format(data[2], '14.10f') + ","
            buffer = buffer + format(data[3], '14.10f') + ","
            buffer = buffer + format(data[4], '14.10f') + ","
            buffer = buffer + format(data[5], '14.10f') + ","
            buffer = buffer + format(data[6], '14.10f') + ","
            buffer = buffer + format(data[7], '14.10f') + "\n"
            self.f_corrimu.write(buffer)
    
    def parse_nmea(self, bytedata):
        if bytedata == 0x24:
            self.nmea_sync = 1
            self.nmea_buffer = []
            self.nmea_buffer.append(bytedata)
        else:
            self.nmea_buffer.append(bytedata)
            if self.nmea_sync == 1:
                if bytedata == 0x0D:
                    self.nmea_sync = 2
            elif self.nmea_sync == 2:
                if bytedata == 0x0A:
                    if len(self.nmea_buffer) > 10 and self.nmea_buffer[-5] == 0x2A:
                        check = nmea_checksum(self.nmea_buffer[1:-2])
                        if check == True:
                            self.f_nmea.write(bytes(self.nmea_buffer))
                self.nmea_buffer = []
                self.nmea_sync = 0

def nmea_checksum(data):
    nmeadata = data[:-3]
    if data[-2] > 0x39:
        tmp1 = data[-2] - 0x37
    else:
        tmp1 = data[-2] - 0x30
    if data[-1] > 0x39:
        tmp2 = data[-1] - 0x37
    else:
        tmp2 = data[-1] - 0x30
    cksum = (tmp1 << 4) + tmp2
    calc_cksum = 0
    for s in nmeadata:
        calc_cksum ^= s
    # print('{0} {1}'.format(cksum, calc_cksum))
    if cksum == calc_cksum:
        return True
    else:
        return False

def calc_crc(payload):
    crc = 0x1D0F
    for bytedata in payload:
        crc = crc ^ (bytedata << 8)
        for i in range(0, 8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1

    crc = crc & 0xffff
    return crc

def mkdir(file_path):
    path = file_path.strip()
    path = path.rstrip("\\")
    path = path[:-4]
    path = path + '_p'
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def receive_args():
    parser = argparse.ArgumentParser()
    parser.description = argparse.ArgumentParser(
        description='Aceinna OpenRTK python parse input args command:')
    parser.add_argument("-p", type=str, help="folder path", default='.')
    parser.add_argument("-i", type=int, help="ins kml rate(hz): 1 2 5 10", default=5)
    return parser.parse_args()


if __name__ == '__main__':
    # compatible code for windows python 3.8
    if is_windows and is_later_py_38:
        import asyncio
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    args = receive_args()

    if args.i != 1 and args.i != 2 and args.i != 5 and args.i != 10:
        print('waring: no ins kml rate {0}, just can be 1 2 5 10!'.format(args.i))
        sys.exit(0)

    for root, dirs, file_name in os.walk(args.p):
        for fname in file_name:
            if (fname.startswith('user') or fname.startswith('ether')) and fname.endswith('.bin'):
                file_path = os.path.join(root, fname)
                print('processing {0}'.format(file_path))
                path = mkdir(file_path)
                if fname.startswith('user'):
                    parse = SerialParse(file_path, path + '/' + fname[:-4] + '_', args.i)
                    parse.start_pasre()
                if fname.startswith('ether'):
                    parse = EtherParse(file_path, path + '/' + fname[:-4] + '_')
                    parse.start_pasre()
