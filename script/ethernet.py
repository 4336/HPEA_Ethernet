import time
import struct
import socket

class Ethernet:
    def __init__(self):
        self.local_ip = '192.168.64.64'  # PC의 IP 주소
        self.local_port = 6401
        self.remote_ip = '192.168.64.128'  # 아두이노의 IP 주소
        self.remote_port = 12801

        self.ping_stamp = int(time.time() * 1000000) % 4294967296

        self.state = 0  # check initialization

        self.tx_packet_struct = struct.Struct('I I B f H H')
        self.rx_packet_struct = struct.Struct('I I B f f f f f f')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rx_packet_struct.size * 100)
        self.sock.bind((self.local_ip, self.local_port))

    def init_connection(self):
        pass  # Any initialization code if needed

    def send_command(self, enable=True, error=False, reset=False, zero=False, valve1=0.0, valve2=0.0, tau=0.0):
        timestamp = int(time.time() * 1000000) % 4294967296

        state = (enable << 0) | (error << 1) | (reset << 2) | (zero << 3)

        valve1 = int(min(max(valve1, 0), 1.0) * 65535)
        valve2 = int(min(max(valve2, 0), 1.0) * 65535)

        packet = self.tx_packet_struct.pack(timestamp, self.ping_stamp, state, tau, valve1, valve2)
        self.sock.sendto(packet, (self.remote_ip, self.remote_port))

    def receive_and_process_data(self, data_processor = None):
        data, _ = self.sock.recvfrom(self.rx_packet_struct.size)
        timestamp, pingstamp, state, pos, vel, tau, pressure1, pressure2, loadcell = self.rx_packet_struct.unpack(data)
        wallstamp = int(time.time() * 1000000) % 4294967296

        self.state = state
        self.ping_stamp = timestamp
        data = {
            'stamp': timestamp,
            'pingstamp': pingstamp,
            'wallstamp': wallstamp,
            'state': state,
            'pos': pos,
            'vel': vel,
            'tau': tau,
            'pressure1': pressure1,
            'pressure2': pressure2,
            'loadcell': loadcell
        }

        # print(f"{wallstamp - pingstamp}, {wallstamp}, {timestamp}, {pingstamp};") # check ping

        if(data_processor):
            control_command = data_processor.create_control_command(data)
            self.send_command(tau=control_command['tau'], valve1=control_command['valve1'], valve2=control_command['valve2'])


            return data, control_command
        else:
            return
