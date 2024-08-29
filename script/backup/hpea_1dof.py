
import time

import scipy.io
import numpy as np
from datetime import datetime

from queue import Queue
# from threading import Lock

from ethernet import Ethernet
from data_process import DataProcessor

def main():
    rx_queue = Queue()
    tx_queue = Queue()
    ethernet = Ethernet(rx_queue, tx_queue)
    data_processor = DataProcessor(rx_queue, tx_queue)

    ethernet.run()

    time.sleep(0.5)
    ethernet.SendCommand(reset=True, enable=False, valve1=1.0, valve2=True)
    time.sleep(0.5)

    while True:
        if ethernet.state == 2:
            break
        time.sleep(0.5)
        print("initializing...", ethernet.state)

    ethernet.SendCommand(valve1=0.0, valve2=0.0)
    time.sleep(0.5)

    print("Start")
    data_processor.run()
    print("Start")

    
    log_state = {} # dict{np.array}
    log_command = {} # dict{np.array}
    enable = True
    for i in range(10):
        log_state.clear()
        log_command.clear()
        cnt = 0
        while True:
            try:
                # print("que:r", uart.ser.in_waiting, rx_queue.qsize(), tx_queue.qsize(), data_processor.data_queue.qsize())
                if not data_processor.data_queue.empty():
                    state, command = data_processor.data_queue.get()

                    for key in state:
                        if key not in log_state:
                            log_state[key] = np.array([state[key]], dtype=np.float64)
                        else:
                            log_state[key] = np.append(log_state[key], state[key])
                    
                    for key in command:
                        if key not in log_command:
                            log_command[key] = np.array([command[key]], dtype=np.float64)
                        else:
                            log_command[key] = np.append(log_command[key], command[key])

                    if len(log_state['stamp']) >= 100*60*10:
                        print("break:", i)
                        break


            except KeyboardInterrupt:
                enable = False

                min_length = min(min(len(values) for values in log_command.values()),
                                min(len(values) for values in log_state.values()))
                
                for key in log_command:
                    log_command[key] = log_command[key][:min_length]
                for key in log_state:
                    log_state[key] = log_state[key][:min_length]
                break
        
        # Save log data
        mat_data = {'state': log_state, 'command': log_command}
        file_name = datetime.now()
        scipy.io.savemat(file_name.strftime("log/Log_%m%d_%H-%M-%S.mat"), mat_data)
        print("dataset", i)
        if not enable:
            break
    
    # Disable
    ethernet.SendCommand(enable=False)


if __name__ == '__main__':
    main()
