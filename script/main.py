import time
import scipy.io
import numpy as np
from datetime import datetime
from ethernet import Ethernet
from data_process import DataProcessor

def main():
    ethernet = Ethernet()
    data_processor = DataProcessor()

    ethernet.init_connection()

    time.sleep(0.5)
    ethernet.send_command(reset=True, enable=False, valve1=1.0, valve2=True)
    time.sleep(0.5)

    while True:
        ethernet.receive_and_process_data()
        if ethernet.state == 2:
            break
        time.sleep(0.1)
        print("initializing...", ethernet.state)

    ethernet.send_command(valve1=0.0, valve2=0.0)
    time.sleep(0.5)

    print("Start")
    
    log_state = {}  # dict{list}
    log_command = {}  # dict{list}
    enable = True

    for i in range(10):
        log_state.clear()
        log_command.clear()
        cnt = 0
        while True:
            try:
                data, control_command = ethernet.receive_and_process_data(data_processor)
                if data:
                    for key in data:
                        if key not in log_state:
                            log_state[key] = [data[key]]  # 리스트로 초기화
                        else:
                            log_state[key].append(data[key])  # 리스트에 추가

                    for key in control_command:
                        if key not in log_command:
                            log_command[key] = [control_command[key]]  # 리스트로 초기화
                        else:
                            log_command[key].append(control_command[key])  # 리스트에 추가

                    if len(log_state['stamp']) >= 1000 * 1000 * 60 * 10:
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

        # 리스트를 NumPy 배열로 변환
        log_state = {key: np.array(log_state[key], dtype=np.float64) for key in log_state}
        log_command = {key: np.array(log_command[key], dtype=np.float64) for key in log_command}

        # Save log data
        mat_data = {'state': log_state, 'command': log_command}
        file_name = datetime.now()
        scipy.io.savemat(file_name.strftime("log/Log_%m%d_%H-%M-%S.mat"), mat_data)
        print("dataset", i)
        if not enable:
            break

    # Disable
    ethernet.send_command(enable=False, valve1=1, valve2=2)

if __name__ == '__main__':
    main()