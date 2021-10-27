import serial
import pynmea2
import os

#ser = serial.Serial('COM3', 230400, timeout=0.1)
ser = open('GPGGA_test.bin', 'rb')
file_size = os.path.getsize('GPGGA_test.bin')

while True:
    #data = str(ser.readline())
    data = str(ser.readline(file_size))
    char = '$'
    nPos = data.find(char)
    if nPos is None:
        pass
    else:
        len_data = len(data)
        nmea_data = data[nPos:]
        if nmea_data.startswith('$GPGGA'):
            gga_data = data[nPos:(len_data-5)]
            print(gga_data)#record.num_sats
        else:
            pass