import time
import struct
import socket
import threading

# from multiprocess_queue import MPQueue
from queue import Queue

class Ethernet:
    def __init__(self, rx_queue: Queue, tx_queue: Queue, baud_rate=1152000):
        self.tic = time.time() # debug

        self.local_ip = '192.168.64.64'  # PC의 IP 주소
        self.local_port = 6401
        self.remote_ip = '192.168.64.128'  # 아두이노의 IP 주소
        self.remote_port = 12801

        self.rx_queue = rx_queue
        self.tx_queue = tx_queue
        
        self.init_time = None

        self.ping_stamp = int(time.time() * 1000000) % 4294967296

        self.state = 0 # check initialization

        self.tx_packet_struct = struct.Struct('I I B f H H')
        self.rx_packet_struct = struct.Struct('I I B f f f f f')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))

        # self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rx_packet_struct.size*2)

    def run(self):
        threading.Thread(target=self._uart_receiver, daemon=True).start()
        threading.Thread(target=self._uart_sender, daemon=True).start()

    def _uart_receiver(self):
        global timestamp
        
        while True:
            tic = time.time()
            data, _ = self.sock.recvfrom(self.rx_packet_struct.size)
            toc = time.time()
            # print(f"Elapsed time: {toc - tic}")
            timestamp, pingstamp, state, pos, vel, tau, p1, p2 = self.rx_packet_struct.unpack(data)
            wallstamp = int(time.time() * 1000000) % 4294967296
            
            # send data to data_processor
            print(timestamp, "state: ", state)
            self.state = state
            self.ping_stamp = pingstamp
            dict = {'stamp': timestamp,
                    'pingstamp': pingstamp,
                    'state': state,
                    'pos': pos,
                    'vel': vel,
                    'tau': tau,
                    'pres1': p1,
                    'pres2': p2}
            # print(dict) # debug
            
            if state == 3:
                self.rx_queue.put([dict, time.time()])
            
            print(f"{wallstamp - pingstamp}, {wallstamp}, {timestamp}, {pingstamp};") # MATLAB
    
    def _uart_sender(self):
        while True:
            try:
                control_command, tic = self.tx_queue.get()
                toc = time.time()
                # print(f"tx time: {toc - tic}")
                self.SendCommand(tau=control_command['tau'], 
                                valve1=control_command['valve1'], 
                                valve2=control_command['valve2'])
            except:
                break

    def SendCommand(self, enable=True, error=False, reset=False, zero=False, \
                    valve1=0.0, valve2=0.0, tau=0.0):
        
        timestamp = int(time.time() * 1000000) % 4294967296

        state = (enable << 0) | (error << 1) | (reset << 2) | (zero << 3)

        valve1 = int(min(max(valve1, 0), 1.0)*65535)
        valve2 = int(min(max(valve2, 0), 1.0)*65535)

        packet = self.tx_packet_struct.pack(timestamp, self.ping_stamp, state, tau, valve1, valve2)
        self.sock.sendto(packet, (self.remote_ip, self.remote_port))
        