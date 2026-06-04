"""
Interpolation controller with pose/joint commands
"""
import os
import threading
import time
import enum
import multiprocessing as mp
import xapi.api as x5
from multiprocessing.managers import SharedMemoryManager
from scipy.spatial.transform import Rotation
from waiting import wait, TimeoutExpired
import numpy as np
from .shared_memory_queue import (SharedMemoryQueue, Empty)
from .shared_memory_ring_buffer import SharedMemoryRingBuffer
from .pose_trajectory_interpolator import (JointTrajectoryInterpolator,                                                                          PoseTrajectoryInterpolator)
from .precise_sleep import precise_wait
from .pose_util import pose_to_mat, mat_to_pose
from .topstar_kine import fkine, ikine, DH_1, DH_2, tx_tip_flange, tx_flange_tip
from math import pi


class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SERVOJ = 2
    SCHEDULE_WAYPOINT = 3
    SETDO = 4
    GETDI = 5

class ActionType(enum.Enum):
    ABS_POSE = 0
    POSE = 1
    JOINT = 2

class GripperType(enum.Enum):
    ECAT = 0
    IO = 1
    VACUUM = 2

class Gripper:
    def __init__(self):
        self.target: float = 0
        self._state: int = 0
        self.type: GripperType

    def on(self):
        pass

    def off(self):
        pass


class VacuumGripper(Gripper):
    def __init__(self, api, out_addr, in_addr):
        super().__init__()
        self.type: GripperType = GripperType.VACUUM
        self.api = api
        self.out_addr = out_addr
        self.in_addr = in_addr
        self.stop = False

    @property
    def state(self):
        self._state = self.api.get_di(self.in_addr)
        return self._state

    def on(self):
        self.api.set_do(self.out_addr, 1)
        self.target = 1

    def off(self):
        self.api.set_do(self.out_addr, 0)
        self.target = 0




class TC:
    V_FUZZ = 1e-6

    def __init__(self, dist, vel, acc):
        self.filter_buf = np.zeros(20)
        self.progress = 0
        self.filter_count = 0
        self.filter_len = 1
        self.current_vel = 0
        self.max_vel = vel
        self.max_acc = acc
        self.cal_progress = 0
        self.target = dist
        self.cal_vel = 0
        self.cycle_time = 0.1

    def run_cycle(self):
        discr = 0.5 * self.cycle_time * self.cal_vel - (self.target - self.cal_progress)
        if discr > 0:
            new_vel = 0.0
        else:
            discr = 0.25 * np.square(self.cycle_time) - 2.0 / self.max_acc * discr
            new_vel = -0.5 * self.max_acc * self.cycle_time + self.max_acc * np.sqrt(discr)
        if np.fabs(new_vel) <= self.V_FUZZ:
            new_vel = 0.0
            new_accel = 0.0
            self.cal_progress = self.target
        else:
            if new_vel > self.max_vel:
                new_vel = self.max_vel
            new_accel = (new_vel - self.cal_vel) / self.cycle_time
            if new_accel > 0.0 and new_accel > self.max_acc:
                new_accel = self.max_acc
                new_vel = self.cal_vel + new_accel * self.cycle_time
            if new_accel < 0.0 and new_accel < -self.max_acc:
                new_accel = -self.max_acc
                new_vel = self.cal_vel + new_accel * self.cycle_time
            self.cal_progress += (new_vel + self.cal_vel) * 0.5 * self.cycle_time
        if self.cal_progress - self.target > self.V_FUZZ:
            self.cal_progress = self.target
        self.cal_vel = new_vel
        filter_vel = self.current_vel
        if self.filter_len <= 1:
            filter_vel = self.cal_vel
        else:
            m = self.filter_count % self.filter_len
            filter_vel += (self.cal_vel - self.filter_buf[m]) / self.filter_len
            self.filter_buf[m] = self.cal_vel
            self.filter_count += 1
        if filter_vel < self.V_FUZZ:
            filter_vel = 0.0
        self.progress += (filter_vel + self.current_vel) * 0.5 * self.cycle_time
        if self.target - self.progress < self.V_FUZZ or filter_vel < self.V_FUZZ:
            self.current_vel = filter_vel
            self.progress = self.target
        else:
            self.current_vel = filter_vel


class RobotAPI:
    def __init__(self, robot_ip, num_joints=6, scale=np.ones((9,))):
        self.cfg: tuple = ()
        self.uf:int = 0
        self.tf:int = 0
        self.vel = 100
        self.acc = 100
        self.cmd_dt = 0.010 # old value: 0.020
        # self.lookahead = 0
        self.gain = 10 # old value: 5
        self.target_pose = np.zeros(num_joints)
        self.target_joints = np.zeros(num_joints)
        self.target_gripper_pos = 0
        self.num_joints = num_joints
        self.scale = scale

        handle = x5.connect(robot_ip)
        # set robot mode
        state = x5.get_system_state(handle)
        if state.remote:
            x5.set_remote(handle, False)
            try:
                wait(lambda: (x5.get_system_state(handle).remote == 0),
                     timeout_seconds=2,
                     sleep_seconds=0.1,
                     waiting_for="机器人本地模式设置成功")
            except TimeoutExpired:
                raise RuntimeError("机器人本地模式设置失败")
        if state.mode != 100:
            x5.enable_servo(handle, False)
            x5.set_system_mode(handle, 100)
            try:
                wait(lambda: (x5.get_system_state(handle).mode == 100),
                     timeout_seconds=2,
                     sleep_seconds=0.1,
                     waiting_for="机器人自动命令模式状态设置成功")
            except TimeoutExpired:
                raise RuntimeError("机器人自动命令模式设置失败")
        if state.enable == 0:
            x5.enable_servo(handle, True)
            count = 0
            while state.enable == 0 and count < 10:
                time.sleep(0.1)
                state = x5.get_system_state(handle)
                count += 1
            if state.enable == 0:
                if state.alarm == 1:
                    error = x5.get_system_alarm_info(handle)
                    print(f"error count：{len(error)}")
                    print(f"error：{error}\n")
                    x5.reset(handle)
                    time.sleep(1.0)
                    state = x5.get_system_state(handle)
                    if state.alarm == 0:
                        x5.enable_servo(handle, True)
                        time.sleep(1.0)
                        state = x5.get_system_state(handle)
                        if state.enable == 0:
                            raise RuntimeError("机器人上使能失败")
                    else:
                        raise RuntimeError("机器人上使能失败")
        self.handle = handle

    def get_target_pose(self):
        return self.target_pose, self.target_gripper_pos

    def get_target_joints(self):
        return self.target_joints, self.target_gripper_pos

    def get_empty_pose(self):
        return np.zeros(6), np.zeros(3)
    
    def get_ee_pose(self):
        try:
            point = x5.get_cpoint(self.handle)
        except x5.RobException as exc:
            print(f"错误代码：{exc.error_code}")
            print(f"错误信息：{exc.error_message}")
            raise Exception()
        pose = point.pose.tolist()
        flange_pose = np.array(pose[:6])
        ext = np.array(pose[6:9]) * self.scale[6:9]
        self.uf = point.uf
        self.tf = point.tf
        self.cfg = tuple([c for c in point.cfg])

        flange_pose[:3] = flange_pose[:3] * 0.001
        flange_pose[3:] = Rotation.from_euler('xyz', flange_pose[3:], degrees=True).as_rotvec()
        tip_pose = mat_to_pose(pose_to_mat(flange_pose) @ tx_flange_tip)
        return tip_pose, ext

    def get_joint_positions(self):
        try:
            c_joint = x5.get_cjoint(self.handle)
        except x5.RobException as exc:
            print(f"错误代码：{exc.error_code}")
            print(f"错误信息：{exc.error_message}")
            raise Exception()
        joint_list = c_joint.tolist()
        joint = np.array([float(temp) for temp in joint_list[:self.num_joints]])*self.scale[:self.num_joints]
        num_ext = 9 - self.num_joints
        ext = (np.array(joint_list[self.num_joints:self.num_joints+num_ext])
               * self.scale[self.num_joints:self.num_joints+num_ext])

        return joint, ext

    def set_do(self, addr: int, state: int):
        try:
            x5.set_do(self.handle, addr, state)
        except x5.RobException as exc:
            print(f"错误代码：{exc.error_code}")
            print(f"错误信息：{exc.error_message}")
            raise Exception()
        return 0

    def get_di(self, addr: int=1):
        try:
            state = x5.get_di(self.handle, addr)
        except x5.RobException as exc:
            print(f"错误代码：{exc.error_code}")
            print(f"错误信息：{exc.error_message}")
            raise Exception()
        return state

    def get_joint_velocities(self):
        return np.zeros(7), np.zeros(2)

    def move_to_joint_positions(self, positions: np.ndarray, gripper_pos, time_to_go: float):
        if time_to_go < 1.0:
            time_to_go = 1.0
        c_joint, c_ext = self.get_joint_positions()
        c_joint = np.concatenate((c_joint, c_ext))
        c_joint /= self.scale
        positions /= self.scale
        # gripper_pos *= 1000.0
        # c_gripper *= 1000.0
        # current = np.append(c_joint, c_gripper)
        # target = np.append(positions, gripper_pos)
        dist = np.fabs(c_joint - positions)
        k = 1.0 / 5.0
        max_dist = np.max(dist)
        v = max_dist / (time_to_go - k)
        tc = TC(max_dist, v, v / k)
        t_start = time.monotonic()
        iter_idx = 0
        while max_dist - tc.progress > 1e-6:
            t_cycle_end = t_start + (iter_idx + 1) * 0.1
            tc.run_cycle()
            ratio = tc.progress / max_dist
            value = (1 - ratio) * c_joint + ratio * positions
            # g_value = (1 - ratio) * c_gripper + ratio * gripper_pos
            joint = x5.Joint(*value[:9])
            try:
                x5.servoj(self.handle, joint, self.cmd_dt, 0, self.gain, self.vel, self.acc)
            except x5.RobException as exc:
                print(f"错误代码：{exc.error_code}")
                print(f"错误信息：{exc.error_message}")
                raise Exception()

            precise_wait(t_cycle_end)
            iter_idx += 1
        print(f'move joint time: {time.monotonic() - t_start}')
        return 0

    def update_desired_joints(self, joints: np.ndarray, gripper_pos: float):
        self.target_joints = joints.copy()
        value = joints / self.scale
        joint = x5.Joint(*value[:9])
        try:
            x5.servoj(self.handle, joint, self.cmd_dt,0, self.gain, self.vel, self.acc)
        except x5.RobException as exc:
            print(f"错误代码：{exc.error_code}")
            print(f"错误信息：{exc.error_message}")
            raise Exception()
        return 0

    def update_desired_ee_pose(self, pose: np.ndarray, gripper_pos: float):
        pos = pose[:3] * 1000.0
        rot = Rotation.from_rotvec(pose[3:6]).as_euler('xyz', degrees=True)
        ext = pose[6:]/self.scale[6:]
        self.target_pose = pose.copy()
        value = x5.Pose(*pos, *rot, *ext)
        point = x5.Point(pose=value, uf=self.uf, tf=self.tf, cfg=self.cfg)
        try:
            x5.servol(self.handle,point, self.cmd_dt, 0, self.gain, self.vel, self.acc)
        except x5.RobException as exc:
            print(f"错误代码：{exc.error_code}")
            print(f"错误信息：{exc.error_message}")
            raise Exception()
        return 0

    def terminate_current_policy(self):
        try:
            x5.stop(self.handle)
            x5.wait_cmd_send_done(self.handle)
        except x5.RobException as exc:
            print(f"错误代码：{exc.error_code}")
            print(f"错误信息：{exc.error_message}")
            raise Exception()
        return 0

    def close(self):
        x5.disconnect(self.handle)


class InterpController(mp.Process):
    """
    To ensure sending command to the robot with predictable latency
    this controller need its separate process (due to python GIL)
    """
    def __init__(self,
                 shm_manager: SharedMemoryManager,
                 frequency=50,
                 launch_timeout=3,
                 joints_init=None,
                 joints_init_duration=None,
                 soft_real_time=False,
                 verbose=False,
                 get_max_k=None,
                 receive_latency=0.0,
                 max_gripper_speed=0.080,
                 max_gripper_width=0.080,
                 action_type=ActionType.JOINT.value,
                 robot_ip=None,
                 num_joints=6,
                 scale=np.ones((9,)),
                 in_addr=None,
                 gripper_addr=None,
                 gripper_enabled=False,
                 ):

        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (9,)

        super().__init__(name="InterpController")
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.joints_init = joints_init
        self.joints_init_duration = joints_init_duration
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose
        self.max_gripper_speed = max_gripper_speed
        self.max_gripper_width = max_gripper_width
        self.action_type = action_type
        self.target_pose = None
        self.target_joints = None
        self.target_gripper = 0
        self.robot_ip = robot_ip
        self.num_joints = num_joints
        self.scale = scale
        self.in_addr = in_addr
        self.gripper_adddr = gripper_addr
        self.gripper_enabled = gripper_enabled

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        # build input queue
        example = {
            'cmd': Command.SERVOL.value,
            'target_pose': np.zeros((9,), dtype=np.float64),
            'gripper_position': 0.0,
            'duration': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=256
        )

        # build ring buffer
        receive_keys = [
            ('ActualTCPPose', 'get_empty_pose'),
            ('ActualQ', 'get_joint_positions'),
            ('ActualQd', 'get_joint_velocities'),
        ]
        example = dict()
        for key, func_name in receive_keys:
            example[key] = np.zeros(9)
        example['TargetTCPPose'] = np.zeros(9)
        example['TargetQ'] = np.zeros(9)
        example['TargetGripper'] = 0
        example['gripper_position'] = 0
        example['robot_receive_timestamp'] = time.perf_counter()
        example['robot_timestamp'] = time.perf_counter()
        example['DI'] = np.zeros(4, dtype=int)
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = ring_buffer
        self.receive_keys = receive_keys

    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[InterpController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.STOP.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()

    def stop_wait(self):
        self.join()

    @property
    def is_ready(self):
        return self.ready_event.is_set()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= command methods ============
    def servoL(self, pose, gripper_pos, duration=0.1):
        """
        duration: desired time to reach pose
        """
        assert self.is_alive()
        assert (duration >= (1 / self.frequency))
        pose = np.array(pose)
        assert pose.shape == (9,)

        message = {
            'cmd': Command.SERVOL.value,
            'target_pose': pose,
            'gripper_position': gripper_pos,
            'duration': duration
        }
        self.input_queue.put(message)

    def servoJ(self, joints, gripper_pos, duration=0.1):
        """
        duration: desired time to reach pose
        """
        assert self.is_alive()
        assert (duration >= (1 / self.frequency))
        joints = np.array(joints)
        assert joints.shape == (9,)

        message = {
            'cmd': Command.SERVOJ.value,
            'target_pose': joints,
            'gripper_position': gripper_pos,
            'duration': duration
        }
        self.input_queue.put(message)
    def schedule_waypoint(self, pose, gripper_pos, target_time):
        pose = np.array(pose)
        # print(f'pose{pose}')
        # print(f'shape{np.shape(pose)}')
        assert pose.shape == (9,)

        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': pose,
            'gripper_position': gripper_pos,
            'target_time': target_time
        }
        self.input_queue.put(message)
    def set_do(self, addr, state):
        message = {
            'cmd': Command.SETDO.value,
            'gripper_position': addr,
            'target_time': state
        }
        self.input_queue.put(message)

    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k, out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    # ========= main loop in process ============
    def run(self):
        try:
            api = RobotAPI(self.robot_ip, self.num_joints, self.scale)
            gripper = (
                VacuumGripper(api, out_addr=self.gripper_adddr[0], in_addr=self.gripper_adddr[1])
                if self.gripper_enabled else None
            )
        except:
            return
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))

        try:
            if self.verbose:
                print(f"[InterpController] robot start running")

            # init pose
            if self.joints_init is not None:
                api.move_to_joint_positions(
                    positions=np.asarray(self.joints_init),
                    gripper_pos=0,  # fully open
                    time_to_go=self.joints_init_duration
                )

            # main loop
            dt = 1. / self.frequency
            curr_joint, ext = api.get_joint_positions()
            curr_joint = np.concatenate((curr_joint, ext))
            curr_pose, ext = api.get_ee_pose()
            curr_pose = np.concatenate((curr_pose, ext))
            curr_gripper = 0

            # use monotonic time to make sure the control loop never go backward
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            if self.action_type == ActionType.JOINT.value:
                joint_interp = JointTrajectoryInterpolator(
                    times=np.array([curr_t]),
                    poses=np.array([curr_joint])
                )
            else:
                pose_interp = PoseTrajectoryInterpolator(
                    times=np.array([curr_t]),
                    poses=np.array([curr_pose])
                )
            gripper_interp = JointTrajectoryInterpolator(
                times=np.array([curr_t]),
                poses=np.array([[curr_gripper]])
            )

            t_start = time.monotonic()
            iter_idx = 0
            keep_running = True
            while keep_running:
                t_now = time.monotonic()
                if self.action_type == ActionType.JOINT.value:
                    joints = joint_interp(t_now)
                else:
                    tip_pose = pose_interp(t_now)
                    pose = mat_to_pose(pose_to_mat(tip_pose[:6]) @ tx_tip_flange)
                    flange_pose = np.concatenate((pose, tip_pose[6:]))
                gripper_width = float(gripper_interp(t_now)[0])

                # send command to robot
                if self.action_type == ActionType.JOINT.value:
                    api.update_desired_joints(joints, 0)
                else:
                    api.update_desired_ee_pose(flange_pose, 0)
                if gripper is not None:
                    if gripper_width > 0.9:
                        gripper.on()
                    elif gripper_width < 0.1:
                        gripper.off()

                # update robot state
                state = dict()
                for key, func_name in self.receive_keys:
                    value, ext = getattr(api, func_name)()
                    state[key] = np.concatenate((value, ext))
                state['DI'] = np.zeros(4, dtype=int)
                if self.in_addr is not None:
                    for i in self.in_addr:
                        state['DI'][i] = api.get_di(i)
                t_recv = time.perf_counter()
                state['robot_receive_timestamp'] = t_recv
                state['robot_timestamp'] = t_recv - self.receive_latency
                state['TargetTCPPose'] = self.target_pose
                state['TargetQ'] = self.target_joints
                state['TargetGripper'] = gripper.target if gripper is not None else 0
                state['gripper_position'] = gripper.state if gripper is not None else 0
                self.ring_buffer.put(state)                    

                # fetch command from queue
                # Drain ALL pending commands at once instead of one per cycle.
                # For SCHEDULE_WAYPOINT we apply latest-wins semantics: only the
                # last waypoint in the batch is executed; earlier ones in the
                # batch are discarded.  This prevents queue back-pressure from
                # turning stale target_times into silent no-ops that freeze the
                # robot when the consumer loop is temporarily slower than the
                # 30 Hz sender.
                try:
                    commands = self.input_queue.get_all()
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                # Pre-scan: index of the last SCHEDULE_WAYPOINT in this batch.
                # Earlier ones will be skipped in the loop below.
                last_schedule_idx = -1
                for _si in range(n_cmd - 1, -1, -1):
                    if int(commands['cmd'][_si]) == Command.SCHEDULE_WAYPOINT.value:
                        last_schedule_idx = _si
                        break

                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']

                    if cmd == Command.STOP.value:
                        keep_running = False
                        # stop immediately, ignore later commands
                        break
                    elif cmd == Command.SETDO.value:
                        addr = int(command['gripper_position'])
                        state = int(command['target_time'])
                        api.set_do(addr, state)
                    elif cmd == Command.SERVOL.value:
                        # since curr_pose always lag behind curr_target_pose
                        # if we start the next interpolation with curr_pose
                        # the command robot receive will have discontinuity
                        # and cause jittery robot behavior.
                        self.target_pose = command['target_pose']
                        self.target_gripper = command['gripper_position']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        pose_interp = pose_interp.drive_to_waypoint(
                            pose=self.target_pose,
                            time=t_insert,
                            curr_time=curr_time,
                        )
                        gripper_interp = gripper_interp.drive_to_waypoint(
                            joints=[self.target_gripper],
                            time=t_insert,
                            max_pos_speed=self.max_gripper_speed,
                            curr_time=curr_time,
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print("[InterpController] New pose target:{} duration:{}s".format(
                                self.target_pose, duration))
                    elif cmd == Command.SERVOJ.value:
                        self.target_joints = command['target_pose']
                        self.target_gripper = command['gripper_position']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        joint_interp = joint_interp.drive_to_waypoint(
                            joints=self.target_joints,
                            time=t_insert,
                            curr_time=curr_time,
                        )
                        gripper_interp = gripper_interp.drive_to_waypoint(
                            joints=[self.target_gripper],
                            time=t_insert,
                            max_pos_speed=self.max_gripper_speed,
                            curr_time=curr_time,
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print(f"[InterpController] New joint target:{self.target_joints} duration:{duration}s")
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        # Latest-wins: skip every SCHEDULE_WAYPOINT except the
                        # last one in the batch.  Earlier ones have the same or
                        # older target_time and would be no-ops anyway once the
                        # queue backs up; discarding them here avoids burning
                        # cycles and keeps last_waypoint_time accurate.
                        if i != last_schedule_idx:
                            continue
                        if self.action_type == ActionType.JOINT.value:
                            self.target_joints = command['target_pose']
                        else:
                            self.target_pose = command['target_pose']
                        self.target_gripper = command['gripper_position']
                        # target_time is already in time.monotonic() domain — sent
                        # as monotonic() by all callers (xapi.py / h1_upper_body.py).
                        target_time = float(command['target_time'])
                        curr_time = t_now + dt
                        if self.action_type == ActionType.JOINT.value:
                            joint_interp = joint_interp.schedule_waypoint(
                                joints=self.target_joints,
                                time=target_time,
                                curr_time=curr_time,
                                last_waypoint_time=last_waypoint_time
                            )
                        else:
                            pose_interp = pose_interp.schedule_waypoint(
                                pose=self.target_pose,
                                time=target_time,
                                curr_time=curr_time,
                                last_waypoint_time=last_waypoint_time
                            )
                        gripper_interp = gripper_interp.schedule_waypoint(
                            joints=[self.target_gripper],
                            time=target_time,
                            max_pos_speed=self.max_gripper_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                # regulate frequency
                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[InterpController] Actual period {(time.monotonic() - t_now):.6f}")

        finally:
            # mandatory cleanup
            # terminate
            print('\nterminate_current_policy\n')
            api.terminate_current_policy()
            api.close()
            self.ready_event.set()
