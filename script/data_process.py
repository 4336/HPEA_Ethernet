import math
import time
import random

class DataProcessor:
    def __init__(self):
        # self.Controller = SignusoidalTorqueTest()
        # self.Controller = SignusoidalValveTest()
        # self.Controller = RandomValveTest()
        # self.Controller = VirtualWall()
        # self.Controller = ImpedanceControl()
        self.Controller = TrajectoryControl()
        # self.Controller = AdmittanceControl()

    def create_control_command(self, processed_data):
        if processed_data['stamp'] > 1000000 * 0.1:  # wait initializations
            control_command = self.Controller.run(processed_data)  # Sinusoidal Torque Test
        else:
            control_command = {
                'stamp': processed_data['stamp'],
                'tau': 0,
                'valve1': 0,
                'valve2': 0
            }
        return control_command

class SignusoidalTorqueTest:
    def __init__(self):
        self.min_torque = 0.0
        self.max_torque = 3.0
        self.period = 3.0

        self.cnt = 0
        self.cnt2 = 0
        self.freq = 1.0 / self.period
        self.CTRL_FREQ = 1000
        self.max_iter = 1000

        self.tau_test = 0.0

    def run(self, data: dict):
        if self.cnt >= self.period * self.CTRL_FREQ:
            self.cnt = 0
            self.cnt2 += 1
            if self.cnt2 >= self.max_iter:
                self.cnt2 = self.max_iter
                return {'stamp': data['stamp'], 'tau': 0, 'valve1': 0, 'valve2': 0}

        self.tau_test = (self.max_torque - self.min_torque) * (
                math.sin(2.0 * math.pi * self.cnt / (self.CTRL_FREQ / self.freq)) / 2.0 + 0.5) + self.min_torque

        self.cnt += 1

        return {'stamp': data['stamp'], 'tau': self.tau_test, 'valve1': 0, 'valve2': 0}

class SignusoidalValveTest:
    def __init__(self):
        self.period = 3.0

        self.cnt = 0
        self.cnt2 = 0
        self.freq = 1.0 / self.period
        self.CTRL_FREQ = 1000
        self.v_test = 0.0
        self.max_iter = 1000

    def run(self, data: dict):
        if self.cnt >= self.period * self.CTRL_FREQ:
            self.cnt = 0
            self.cnt2 += 1
            if self.cnt2 >= self.max_iter:
                self.cnt2 = self.max_iter
                return {'stamp': data['stamp'], 'tau': 0, 'valve1': 0, 'valve2': 0}

        self.v_test = math.sin(2.0 * math.pi * self.cnt / (self.CTRL_FREQ / self.freq)) / 2.0 + 0.5

        self.cnt += 1

        return {'stamp': data['stamp'], 'tau': 0, 'valve1': 0, 'valve2': self.v_test}

class RandomValveTest:
    def __init__(self):
        self.period = 3.0

        self.cnt = 0
        self.cnt2 = 0
        self.freq = 1.0 / self.period
        self.CTRL_FREQ = 1000
        self.v_test = 0.0
        self.v_period = 4
        self.max_iter = 1000

    def run(self, data: dict):
        if self.cnt >= self.period * self.CTRL_FREQ:
            self.cnt = 0
            self.cnt2 += 1
            if self.cnt2 >= self.max_iter:
                self.cnt2 = self.max_iter
                return {'stamp': data['stamp'], 'tau': 0, 'valve1': 0, 'valve2': 0}

        if(self.cnt == 0):
            self.v_test = 1.0
            print(self.v_period)
        elif(self.cnt >= self.v_period):
            self.v_test = 0.0
        
        self.cnt += 1

        return {'stamp': data['stamp'], 'tau': 0, 'valve1': 0, 'valve2': self.v_test}
    

class VirtualWall:
    def __init__(self, wall_pos = 0.31415926, K_Npm = 100, torque_const = 0.105, arm_length = 0.3):
        self.wall_pos = wall_pos  # 벽 위치
        self.K_Npm = K_Npm  # 강성 [N/m]
        self.torque_const = torque_const  # 토크 상수 [Nm/A]
        self.arm_length = arm_length  # 모멘트암 길이 [m]

    def run(self, data: dict):
        pos = data['pos'] * self.arm_length

        # Virtual wall의 힘 계산
        if pos >= self.wall_pos:
            force = -self.K_Npm * (pos - self.wall_pos)  # 벽을 통과할 수 없도록 힘을 계산
        else:
            force = 0.0  # 벽 안쪽에 있을 때는 힘이 작용하지 않음

        # 모터 출력 토크 계산
        tau = force * self.arm_length # / self.torque_const

        return {'stamp': data['stamp'], 'tau': tau, 'valve1': 0, 'valve2': 0}

class LowPassFilter:
    def __init__(self, cutoff_hz = 10, sampling_hz = 1000):
        self.cutoff = cutoff_hz
        self.hz = sampling_hz
        self.dt = 1/self.hz
        self.omega = 2*math.pi*self.cutoff # cutoff freqency
        self.beta = math.exp(-self.omega * self.dt)
        self.alpha = 1-self.beta

        self.output = None
    
    def run(self, data):
        if self.output == None:
            self.output = data

        self.output = (1-self.alpha) * self.output + self.alpha * data
        
        return self.output

class ImpedanceControl:
    def __init__(self, wall_pos1 = 10, wall_pos2 = 110, MBK = [0, 1, 3000], sampling_rate = 1000):
        self.arm_length = 0.3 # moment arm [m]
        self.dt = 1/sampling_rate

        self.M_Ns2pm = MBK[0] # [Ns^2/m]
        self.B_Nspm = MBK[1] # [Ns/m]
        self.K_Npm = MBK[2] # [N/m]

        self.wall_pos1 = wall_pos1 * (math.pi/180) * self.arm_length # 벽 위치 [m]
        self.wall_pos2 = wall_pos2 * (math.pi/180) * self.arm_length # 벽 위치 [m]

        self.prev_time = 0
        self.prev_vel = None
        self.velLPF = LowPassFilter(100, sampling_rate)
        self.accLPF = LowPassFilter(1, sampling_rate)

    def run(self, data: dict):
        timestamp = data['stamp'] / 1000000.0 # [us] to [s]
        dt = timestamp - self.prev_time
        if dt < self.dt/3 or self.dt*3 <dt:
            print("dt:", dt)
            dt = self.dt

        pos = data['pos'] * self.arm_length
        vel_raw = data['vel'] * self.arm_length
        vel = self.velLPF.run(vel_raw)

        if(self.prev_vel == None):
            self.prev_vel = vel
        acc_raw = (vel - self.prev_vel) / dt
        acc = self.accLPF.run(acc_raw)
        # print(acc_raw, acc,";")

        force_M = self.M_Ns2pm * acc

        force_B = self.B_Nspm * vel

        # Virtual wall
        if pos <= self.wall_pos1:
            force_K = self.K_Npm * (pos - self.wall_pos1)  # 벽을 통과할 수 없도록 힘을 계산
        elif self.wall_pos2 <= pos:
            force_K = self.K_Npm * (pos - self.wall_pos2)
        else:
            force_K = 0.0  # 벽 안쪽에 있을 때는 힘이 작용하지 않음

        # force to torque
        force = force_M + force_B + force_K
        tau = -force * self.arm_length

        # buffer
        self.prev_time = timestamp
        self.prev_vel = vel

        return {'stamp': data['stamp'], 'tau': tau, 'valve1': 0, 'valve2': 0}
    
class PIDControl:
    def __init__(self, Kp = 1.0, Ki = 0.0, Kd = 0.0, dt = 0.001, i_max = 100.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.dt = dt

        self.i_max = i_max

        self.prev_error = 0
        self.integral = 0
    
    def run(self, error, reset = False):
        if reset:
            self.integral = 0
            self.prev_error = error
        
        self.integral += error
        self.integral = min(max(self.Ki * self.integral, -self.i_max), self.i_max) / self.Ki # anti-windup

        print("integ:", self.Ki * self.integral)

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    
class TrajectoryControl:
    def __init__(self, wall_pos1_deg = 10, wall_pos2_deg = 110, sampling_rate = 1000):
        self.arm_length = 0.3 # moment arm [m]
        self.dt = 1/sampling_rate

        self.wall_pos1 = wall_pos1_deg * (math.pi/180) * self.arm_length # 벽 위치 [m]
        self.wall_pos2 = wall_pos2_deg * (math.pi/180) * self.arm_length # 벽 위치 [m]

        self.prev_time = 0

        self.velLPF = LowPassFilter(100, sampling_rate)

        self.initPID = PIDControl(10.0, 1, 3.0, self.dt, i_max = 5.0)
        self.PID = PIDControl(1000.0, 1, 3.0, self.dt, i_max = 5.0)

        self.time_step = 0.0
        self.time_step_offset = 0.0

        self.init_time = sampling_rate * 1 # wait time for initialization (1s)
        self.init_cnt = self.init_time
        self.hz = 4.0
        self.traj_magnitude = self.wall_pos2 - self.wall_pos1
        self.traj_offset = (self.traj_magnitude / 2) + self.wall_pos1

    def run(self, data: dict, reset = False):
        if reset:
            self.init_cnt = self.init_time

        timestamp = data['stamp'] / 1000000.0 # [us] to [s]
        dt = timestamp - self.prev_time
        if dt < self.dt/3 or self.dt*3 <dt:
            print("dt:", dt)
            dt = self.dt

        pos = data['pos'] * self.arm_length # joint to cartesian
        vel_raw = data['vel'] * self.arm_length # joint to cartesian
        vel = self.velLPF.run(vel_raw)


        # Trajectory Generation
        if self.init_cnt > 0:
            if abs(pos - self.wall_pos1) < 0.003:
                self.init_cnt -= 1
                alpha = 0.9
                self.initPID.Kp = self.initPID.Kp * alpha + self.PID.Kp * (1 - alpha)
                self.initPID.Ki = self.initPID.Ki * alpha + self.PID.Ki * (1 - alpha)
                self.initPID.Kd = self.initPID.Kd * alpha + self.PID.Kd * (1 - alpha)
            
            self.time_step_offset = timestamp
            force = self.initPID.run(self.wall_pos1 - pos)
            print("force:", force)
        
        else:
            self.time_step = timestamp - self.time_step_offset
            target_pos = - (self.traj_magnitude / 2) * math.cos(2 * math.pi * self.time_step / self.hz) + self.traj_offset

            force = self.PID.run(target_pos - pos)

        tau = force * self.arm_length

        # buffer
        self.prev_time = timestamp

        return {'stamp': data['stamp'], 'tau': tau, 'valve1': 0, 'valve2': 0}
    
    
class AdmittanceControl:
    def __init__(self, wall_pos1 = 10, wall_pos2 = 110, MBK = [1, 1, 1000], sampling_rate = 1000):
        self.arm_length = 0.3 # moment arm [m]
        self.dt = 1/sampling_rate

        self.M_Ns2pm = MBK[0] # 관성 [Ns^2/m]
        self.B_Nspm = MBK[1] # 감쇠 [Ns/m]
        self.K_Npm = MBK[2] # 강성 [N/m]

        self.wall_pos1 = wall_pos1 * (math.pi/180) * self.arm_length # 벽 위치 [m]
        self.wall_pos2 = wall_pos2 * (math.pi/180) * self.arm_length # 벽 위치 [m]

        self.prev_time = 0

        self.adm_pos = None
        self.adm_vel = None
        self.acm_acc = None

        self.velLPF = LowPassFilter(100, sampling_rate)

        self.PID = PIDControl(1000.0, 0.0, 3.0, self.dt)

    def run(self, data: dict):
        timestamp = data['stamp'] / 1000000.0 # [us] to [s]
        dt = timestamp - self.prev_time
        if dt < self.dt/3 or self.dt*3 <dt:
            print("dt:", dt)
            dt = self.dt

        pos = data['pos'] * self.arm_length # joint to cartesian
        vel_raw = data['vel'] * self.arm_length # joint to cartesian
        vel = self.velLPF.run(vel_raw)

        if self.adm_pos == None:
            self.adm_pos = pos
        if self.adm_vel == None:
            self.adm_vel = vel

        load = data['loadcell']

        # Virtual wall
        if pos <= self.wall_pos1:
            force_K = self.K_Npm * (pos - self.wall_pos1)  # 벽을 통과할 수 없도록 힘을 계산
        elif self.wall_pos2 <= pos:
            force_K = self.K_Npm * (pos - self.wall_pos2)
        else:
            force_K = 0.0  # 벽 안쪽에 있을 때는 힘이 작용하지 않음

        # force to torque
        self.acm_acc = (load - self.B_Nspm * self.adm_vel - force_K) / self.M_Ns2pm
        self.adm_vel = self.adm_vel + self.acm_acc * dt
        self.adm_pos = self.adm_pos + self.adm_vel * dt
        print("pos:",self.adm_pos, "load:", load)
        
        # self.adm_pos = math.pi/3 * self.arm_length # test position controller

        force = self.PID.run(self.adm_pos - pos)
        
        tau = force * self.arm_length

        # buffer
        self.prev_time = timestamp

        return {'stamp': data['stamp'], 'tau': tau, 'valve1': 0, 'valve2': 0}
    