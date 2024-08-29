import serial
import threading
import struct
import glob
import copy
import time
import ctypes
import platform

# from multiprocess_queue import MPQueue
from queue import Queue
from threading import Lock

STX = 0x02
ETX = 0x03
UART_TX_LEN = 15
UART_RX_LEN = 21

class UART:
    def __init__(self, rx_queue: Queue, tx_queue: Queue, baud_rate=1152000):
        self.tic = time.time()

        self.rx_queue = rx_queue
        self.tx_queue = tx_queue
        
        self.init_time = None

        self.state = 0 # check initialization

        self.packet_init = False
        self.rx_buf = bytearray(UART_RX_LEN)
        self.tx_buf = bytearray(UART_TX_LEN)
        
        port = self.find_usbmodem_port()
        if not port:
            raise IOError("No USB modem port found")
        self.ser = serial.Serial(port, baud_rate, timeout=1)

    def run(self):
        threading.Thread(target=self._uart_receiver, daemon=True).start()
        threading.Thread(target=self._uart_sender, daemon=True).start()

    @staticmethod
    def find_usbmodem_port():
        system = platform.system()
        port = None
        if system == 'Windows':
            ports = glob.glob('COM3')
        elif system == 'Darwin':
            ports = glob.glob('/dev/tty.usbmodem*')
        elif system == 'Linux':
            ports = glob.glob('/dev/ttyUSB[1-9]*')
        
        port = ports[0] if ports else None
        print("port:",port)
        return port

    def _uart_receiver(self):
        while True:
            try:
                rx = self.RX_Packet()
                if rx:
                    self.ParsePacket(rx)
            except serial.SerialException:
                break
    
    def _uart_sender(self):
        while True:
            try:
                # if self.tx_queue.qsize() > 0:
                control_command = self.tx_queue.get()
                if True:
                    self.SendCommand(tau=control_command['tau'], 
                                    valve1=control_command['valve1'], 
                                    valve2=control_command['valve2'])
                        
            except serial.SerialException:
                break

    def RX_Packet(self):
        # if self.init_time is not None:
        #     with self.print_lock:
        #         print(time.time() - self.init_time, "wait:", self.ser.in_waiting)
        if not self.packet_init:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte[0] == STX:
                    self.rx_buf[0] = byte[0]
                    self.packet_init = True
                else:
                    print("STX Error")
        elif self.ser.in_waiting >= UART_RX_LEN - 1:
            self.rx_buf[1:] = self.ser.read(UART_RX_LEN - 1)
            crc_get = struct.unpack('<H', self.rx_buf[UART_RX_LEN - 3:UART_RX_LEN - 1])[0]
            crc_make = self.make_crc16(self.rx_buf[:UART_RX_LEN - 3])
            if crc_get == crc_make and self.rx_buf[UART_RX_LEN - 1] == ETX: # normal check option
            # if self.rx_buf[UART_RX_LEN - 1] == ETX: # fast check option
                self.packet_init = False

                # # print tic
                # if self.tic is None:
                #     self.tic = time.time()
                # toc = time.time()
                # print("dt:", toc - self.tic)
                # self.tic = toc

                return copy.deepcopy(self.rx_buf)
            else:
                self.packet_init = False
                print("CRC Error")
        return False

    def ParsePacket(self, rx):
        # print(rx)
        stamp = struct.unpack('<H', rx[1:3])[0]
        if self.init_time == None:
            self.init_time = time.time()
            
            self.init_stamp = stamp
            self.stamp_overflow = 0
            self.stamp = stamp
        if self.stamp > stamp + 0x10000 * self.stamp_overflow:
            self.stamp_overflow += 1
            # print(self.stamp, stamp, self.stamp_overflow)
        stamp += 0x10000 * self.stamp_overflow - self.init_stamp
        self.stamp = stamp

        state = ctypes.c_int8(rx[3]).value
        self.state = state

        valve1 = struct.unpack('<H', rx[4:6])[0] / 65535.0
        valve2 = struct.unpack('<H', rx[6:8])[0] / 65535.0

        pos = struct.unpack('<h', rx[8:10])[0] / 100.0
        vel = struct.unpack('<h', rx[10:12])[0] / 100.0
        tau = struct.unpack('<h', rx[12:14])[0] / 100.0
        pres1 = struct.unpack('<h', rx[14:16])[0] / 10.0
        pres2 = struct.unpack('<h', rx[16:18])[0] / 10.0

        dict = {'stamp': stamp, 
                'state': state, 
                'valve1': valve1, 
                'valve2': valve2, 
                'pos': pos, 
                'vel': vel, 
                'tau': tau, 
                'pres1': pres1,
                'pres2': pres2}
        
        # print(dict)
        
        if state == 3:
            self.rx_queue.put(dict)

    def SendCommand(self, enable=True, error=False, reset=False, zero=False, \
                    valve1=0.0, valve2=0.0, tau=0.0):
        
        self.tx_buf[0] = STX
        timestamp = int(time.time() * 1000) & 0xFFFF
        self.tx_buf[1:3] = int(timestamp).to_bytes(2, 'little')
        
        # 명령어 바이트 생성
        command_byte = 0
        command_byte |= (enable << 0)
        command_byte |= (error << 1)
        command_byte |= (reset << 2)
        command_byte |= (zero << 3)
        self.tx_buf[3] = command_byte

        self.tx_buf[4:6] = int(min(max(valve1, 0), 1.0)*65535).to_bytes(2, 'little')
        self.tx_buf[6:8] = int(min(max(valve2, 0), 1.0)*65535).to_bytes(2, 'little')
        
        tau_limit = min(max(tau, -10.0), 10.0)
        self.tx_buf[8:12] = struct.pack('<f', tau_limit)
        
        self.tx_buf[12:14] = self.make_crc16(self.tx_buf[0:12]).to_bytes(2, 'little')

        self.tx_buf[14] = ETX

        self.ser.write(self.tx_buf)
        # print(self.tx_buf)

    @staticmethod
    def make_crc16(data):
        crc = 0xFFFF
        for byte in data:
            crc = ((crc << 8) ^ CCITT_Table[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
        return crc

CCITT_Table = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]
