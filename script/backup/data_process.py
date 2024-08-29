#!/usr/bin/env python3.11
import math
import random
import threading
# from multiprocessing import Manager
# from concurrent.futures import ProcessPoolExecutor

# Custom imports
# from multiprocess_queue import MPQueue
from queue import Queue
import time


class DataProcessor:
    def __init__(self, rx_queue: Queue, tx_queue: Queue):
        self.rx_queue = rx_queue
        self.tx_queue = tx_queue

        self.shared_data = Queue()
        self.data_queue = Queue()

        self.SinTorqueTest = SignusoidalTorqueTest()

    def run(self):
        # while self.rx_queue.qsize() > 0: # Clear the queue
        #     self.rx_queue.get()
        self.process_thread = threading.Thread(target=self._process_data, daemon=True).start()
    
    def _process_data(self):
        while True:
            qsize = self.rx_queue.qsize()
            if qsize > 10:
                print("Data queue: ", qsize)
            # if qsize > 0:
            data, tic = self.rx_queue.get()
            toc = time.time()
            # print(f"rx time: {toc - tic}")
            # self.shared_data.put(data)

            if data:
                control_command = self.create_control_command(data)
                self.tx_queue.put([control_command, time.time()])
                self.data_queue.put((data, control_command)) # Save data and control command
                # print("control_command:", control_command) # Debugging

    def create_control_command(self, processed_data):
        if processed_data['stamp'] > 1000000 * 0.1: # wait initializations
            control_command = self.SinTorqueTest.run(processed_data) # Sinusoidal Torque Test
        else:
            valve1 = 0
            valve2 = 0
            tau = 0
            control_command = dict(stamp=processed_data['stamp'], 
                                   tau=tau, 
                                   valve1=valve1, 
                                   valve2=valve2)
        return control_command

class SignusoidalTorqueTest:
    def __init__(self):
        self.min_torque = 0.0
        self.max_torque = 1.0
        self.period = 3.0

        self.cnt = 0
        self.cnt2 = 0
        self.freq = 1.0 / self.period
        self.CTRL_FREQ = 1000
        # self.v_test = False
        self.v_test = 0.0
    
    def run(self, data:dict):
        if self.cnt >= self.period * self.CTRL_FREQ:
            self.cnt = 0
            self.cnt2 += 1
            if self.cnt2 >= self.CTRL_FREQ:
                self.cnt2 = self.CTRL_FREQ
                return dict(tau=0, valve1=False, valve2=False)

        self.v_test = (self.max_torque - self.min_torque) * (math.sin(2.0 * math.pi * self.cnt / (self.CTRL_FREQ / self.freq)) / 2.0 + 0.5) + self.min_torque
 
        self.cnt += 1

        return dict(stamp=data['stamp'], tau=0, valve1=self.v_test, valve2=False)