import threading
import queue
import time

# 큐 생성
a_queue = queue.Queue()
b_queue = queue.Queue()

# 데이터 전송 함수 정의
def thread_A(a_queue, b_queue):
    while True:
        # B 쓰레드로부터 데이터 수신
        data = b_queue.get()
        start_time = time.time()
        # B 쓰레드로 데이터 전송
        a_queue.put(start_time)

def thread_B(a_queue, b_queue):
    while True:
        # A 쓰레드로부터 데이터 수신
        start_time = a_queue.get()
        # A 쓰레드로 데이터 전송
        b_queue.put(start_time)

# 쓰레드 생성 및 시작
thread_a = threading.Thread(target=thread_A, args=(a_queue, b_queue))
thread_b = threading.Thread(target=thread_B, args=(a_queue, b_queue))

thread_a.start()
thread_b.start()

# RTT 측정
while True:
    time.sleep(1)  # 1초 간격으로 RTT 측정
    send_time = time.time()
    b_queue.put(send_time)
    round_trip_time = a_queue.get() - send_time
    print(f"RTT: {round_trip_time * 1000:.3f} ms")