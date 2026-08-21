'''
实验名称：串口多任务位置控制 + Wi-Fi网页图传 (双线程融合版)
实验平台：01Studio CanMV K230

功能说明：
    1. 自动稳定小球并响应串口多任务指令 (任务3, 4, 5, 6)。
    2. 开启 Wi-Fi AP 热点 (SSID: GK-TFBOSS-ds)。
    3. 浏览器访问 192.168.169.1:8081 可查看实时无UI的灰度局部视频流并录像。
'''

import time, os, gc, _thread
import network, socket
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART
from machine import FPIOA

# ============================================================
# 1. 图传与网络参数
# ============================================================
AP_SSID = "GK-TFBOSS-ds"
AP_KEY = "12345678"
HTTP_PORT = 8081
STREAM_BOUNDARY = "frame"
JPEG_QUALITY = 35
TARGET_FPS = 15

# ============================================================
# 2. 视觉检测与流媒体区域参数
# ============================================================
BALL_THRESHOLDS = [(71, 91)]

IMG_W, IMG_H = 800, 480
LCD_W, LCD_H = 800, 480

# 小球尺寸筛选参数，防止反光、管道边缘等目标被误识别
BALL_MIN_W = 8
BALL_MAX_W = 38
BALL_MIN_H = 8
BALL_MAX_H = 38
BALL_MIN_PIXELS = 50
BALL_MAX_PIXELS = 1200
BALL_MIN_ASPECT = 0.55
BALL_MAX_ASPECT = 1.80

# PD控制检测区域
ROI_HEIGHT = 40
ROI = (25, (IMG_H - ROI_HEIGHT) // 2, IMG_W-75, ROI_HEIGHT)

# 图传专属截取区域 (仅截取中间 800x128 凹槽区域，节约网络带宽)
STREAM_W = IMG_W
STREAM_H = 128
STREAM_ROI = (0, (IMG_H - STREAM_H) // 2, STREAM_W, STREAM_H)

# ============================================================
# 3. PD控制与目标参数
# ============================================================
CENTER_X = 378
PIXELS_PER_CM = 26.0

POSITIVE_TARGET_X = 513
NEGATIVE_TARGET_X = 234

DEADZONE = 30
STATIC_KP = 0.6
STATIC_KD = 4.0
STATIC_KI = 0.0
TASK3_NEGATIVE_KD = 4.8
DRIVE_KP = 0.6
DRIVE_KD = 4.0
DRIVE_KI = 0.0
MAX_PULSE = 60
DRIVE_MAX_PULSE = 180
TASK3_TO_POSITIVE_MAX_PULSE = 70
TASK3_NEGATIVE_MAX_PULSE = 50
TASK3_APPROACH_REVERSE_BRAKE_MAX_PULSE = 20
TASK3_OVERSHOOT_RETURN_MAX_PULSE = 15
MIN_PULSE = 55
TASK3_POSITIVE_MIN_PULSE = 50
TASK4_LAUNCH_COMPENSATION_PULSE = 50
TASK4_STOP_COMPENSATION_PULSE = -20
TASK4_LAUNCH_COMPENSATION_MS = 1500
TASK4_STOP_COMPENSATION_MS = 500
TASK5_LAUNCH_COMPENSATION_PULSE = 50
TASK5_LAUNCH_COMPENSATION_MS = 3000
TASK5_DECEL_COMPENSATION_PULSE = -20
TASK5_DECEL_COMPENSATION_MS = 3000
TASK4_COMPENSATION_RELEASE_STEP = 20
TASK3_FINAL_TOLERANCE_PX = int(PIXELS_PER_CM * 1.0)
TASK3_POSITIVE_TOLERANCE_PX = int(PIXELS_PER_CM * 0.5)
TASK3_STATIONARY_DELTA_PX = 2
CONTROL_INTERVAL = 50
TILT_SPEED = 300
MOTOR_MID_POSITION = 0
TARGET_STABLE_FRAMES = 5
BALL_LOST_STOP_TIME = 100
INTEGRAL_UPDATE_DIVIDER = 10
INTEGRAL_LIMIT = 500.0
CONTROL_DEADZONE = 5

# ============================================================
# 4. 运行状态定义
# ============================================================
STATE_IDLE = 0
STATE_TASK3_TO_CENTER = 30
STATE_TASK3_TO_POSITIVE = 31
STATE_TASK3_TO_NEGATIVE = 32
STATE_TASK3_HOLD_NEGATIVE = 33
STATE_TASK4_LAUNCH_COMPENSATION = 40
STATE_TASK4_COMPENSATION_RELEASE = 41
STATE_TASK4_STOP_COMPENSATION = 42
STATE_HOLD_CENTER = 45
STATE_TASK5_DECEL_COMPENSATION = 50
STATE_TASK6_CAPTURE_POSITION = 60
STATE_TASK6_HOLD_POSITION = 61
STATE_TASK6_ARMED = 62

# 串口协议状态
RX_WAIT_HEADER = 0
RX_WAIT_TASK = 1
RX_WAIT_END = 2

# ============================================================
# 5. 全局共享变量
# ============================================================
# 线程间通信共享的最新图像
latest_img = None
thread_lock = _thread.allocate_lock()
thread_running = False

# 硬件对象
uart = None
sensor = None

# 控制状态全局变量
active_task = 0
motion_state = STATE_HOLD_CENTER
target_x = CENTER_X
stable_frame_count = 0
previous_error = 0
pd_initialized = False
balance_integral = 0.0
balance_integral_output = 0.0
balance_integral_counter = 0
balance_target_x = CENTER_X
motor_position_output = None
task4_compensation_start_time = 0
rx_state = RX_WAIT_HEADER
rx_task_id = 0
last_cmd_time = 0

# ============================================================
# 6. 电机与任务控制函数
# ============================================================
def get_motor_position_limit():
    if motion_state in (STATE_TASK4_LAUNCH_COMPENSATION,
                        STATE_TASK4_STOP_COMPENSATION):
        return DRIVE_MAX_PULSE
    if motion_state == STATE_TASK3_TO_POSITIVE:
        return TASK3_TO_POSITIVE_MAX_PULSE
    if active_task in (4, 5, 6):
        return DRIVE_MAX_PULSE
    return MAX_PULSE

def get_pid_gains():
    if motion_state in (STATE_TASK3_TO_NEGATIVE,
                        STATE_TASK3_HOLD_NEGATIVE):
        return STATIC_KP, TASK3_NEGATIVE_KD, STATIC_KI
    if active_task in (4, 5, 6):
        return DRIVE_KP, DRIVE_KD, DRIVE_KI
    return STATIC_KP, STATIC_KD, STATIC_KI

def stop_motor():
    if uart: uart.write(bytes([0x04, 0xFE, 0x98, 0x00, 0x6B]))

def control_motor_absolute_position(position, speed_rpm):
    if not uart:
        return False

    # 张大头绝对位置模式：
    # 方向字节表示绝对坐标正负，pulse_val表示绝对位置大小。
    dir_byte = 0x00 if position >= 0 else 0x01
    pulse_val = min(int(abs(position)), 0xFFFFFFFF)
    speed_val = int(abs(speed_rpm))

    p3, p2, p1, p0 = (pulse_val >> 24) & 0xFF, (pulse_val >> 16) & 0xFF, (pulse_val >> 8) & 0xFF, pulse_val & 0xFF
    s_h, s_l = (speed_val >> 8) & 0xFF, speed_val & 0xFF
    uart.write(bytes([0x04, 0xFD, dir_byte, s_h, s_l, 0x0F, p3, p2, p1, p0, 0x01, 0x00, 0x6B]))
    return True

def balanceX(position):
    global previous_error, pd_initialized
    global balance_integral, balance_integral_output
    global balance_integral_counter, balance_target_x

    # 与板球系统balanceX结构一致：
    # 位置偏差 -> P、D及低频I -> 有限的电机位置输出
    kp, kd, ki = get_pid_gains()
    bias = position - target_x

    task3_hold_deadzone = (TASK3_FINAL_TOLERANCE_PX
                           if motion_state == STATE_TASK3_HOLD_NEGATIVE
                           else CONTROL_DEADZONE)
    # 中心附近不再累计积分，摆杆回到机械水平中位，避免微小误差越积越大。
    if abs(bias) <= task3_hold_deadzone:
        previous_error = bias
        pd_initialized = True
        balance_integral = 0.0
        balance_integral_output = 0.0
        balance_integral_counter = 0
        return 0

    # 目标点改变时重新初始化，避免目标切换产生微分冲击
    if balance_target_x != target_x:
        balance_target_x = target_x
        previous_error = bias
        pd_initialized = True
        balance_integral = 0.0
        balance_integral_output = 0.0
        balance_integral_counter = 0

    if pd_initialized:
        differential = bias - previous_error
    else:
        differential = 0.0
        pd_initialized = True

    # 参考板球程序，积分降低更新频率，并进行积分限幅
    balance_integral_counter += 1
    if balance_integral_counter > INTEGRAL_UPDATE_DIVIDER:
        balance_integral_counter = 0
        balance_integral += bias

        if balance_integral < -INTEGRAL_LIMIT:
            balance_integral = -INTEGRAL_LIMIT
        elif balance_integral > INTEGRAL_LIMIT:
            balance_integral = INTEGRAL_LIMIT

        balance_integral_output = ki * balance_integral

    task3_static_boost = False
    balance = (
        kp * bias
        + kd * differential
        + balance_integral_output
    )

    # Task 3 positive leg uses a tighter target band. If static friction
    # stops the ball early, apply the minimum useful correction output.
    if motion_state == STATE_TASK3_TO_POSITIVE:
        if (abs(bias) > TASK3_POSITIVE_TOLERANCE_PX
                and abs(differential) <= TASK3_STATIONARY_DELTA_PX
                and abs(balance) < TASK3_POSITIVE_MIN_PULSE):
            balance = (TASK3_POSITIVE_MIN_PULSE
                       if bias > 0 else -TASK3_POSITIVE_MIN_PULSE)

    # 任务3前往/保持-5cm时，为静止且未进入合规区的钢球补足最小有效倾角。
    if motion_state in (STATE_TASK3_TO_NEGATIVE, STATE_TASK3_HOLD_NEGATIVE):
        if (abs(bias) > TASK3_FINAL_TOLERANCE_PX
                and abs(differential) <= TASK3_STATIONARY_DELTA_PX
                and abs(balance) < MIN_PULSE):
            balance = MIN_PULSE if bias > 0 else -MIN_PULSE
            task3_static_boost = True

    previous_error = bias
    output_limit = get_motor_position_limit()
    if (motion_state in (STATE_TASK3_TO_NEGATIVE,
                         STATE_TASK3_HOLD_NEGATIVE)
            and not task3_static_boost):
        output_limit = min(output_limit, TASK3_NEGATIVE_MAX_PULSE)

    if balance < -output_limit:
        balance = -output_limit
    elif balance > output_limit:
        balance = output_limit
    # Before reaching -5 cm (X > target), D may request a reverse tilt to
    # brake the ball. Keep that braking action gentle so it cannot send the
    # ball back toward -3 cm before reaching the target.
    if (motion_state in (STATE_TASK3_TO_NEGATIVE,
                         STATE_TASK3_HOLD_NEGATIVE)
            and bias > 0
            and balance < -TASK3_APPROACH_REVERSE_BRAKE_MAX_PULSE):
        balance = -TASK3_APPROACH_REVERSE_BRAKE_MAX_PULSE
    if (motion_state in (STATE_TASK3_TO_NEGATIVE,
                         STATE_TASK3_HOLD_NEGATIVE)
            and bias < 0
            and not task3_static_boost
            and balance < -TASK3_OVERSHOOT_RETURN_MAX_PULSE):
        balance = -TASK3_OVERSHOOT_RETURN_MAX_PULSE

    # 反积分饱和：输出被截断时，反算积分值，防止积分继续"借债"
    if ki != 0:
        balance_integral = (balance - kp * bias - kd * differential) / ki
        balance_integral_output = ki * balance_integral

    return int(balance)

def set_motor_position(target_output):
    global motor_position_output

    # 模拟板球舵机“中点 + 控制偏移量”的绝对位置输出：
    # 舵机为1500 + motor_x，步进电机为MOTOR_MID_POSITION + target_output。
    output_limit = get_motor_position_limit()
    if target_output < -output_limit:
        target_output = -output_limit
    elif target_output > output_limit:
        target_output = output_limit

    absolute_position = int(MOTOR_MID_POSITION + target_output)

    # 目标绝对位置改变时才发送，避免重复占用串口。
    if motor_position_output is None or absolute_position != motor_position_output:
        if control_motor_absolute_position(absolute_position, TILT_SPEED):
            motor_position_output = absolute_position

    # UI中的P显示相对中点的PID位置输出，与板球舵机PWM偏移量一致。
    return int(target_output)

def set_motor_position_smooth(target_output, max_step):
    global motor_position_output

    current_output = 0 if motor_position_output is None else int(
        motor_position_output - MOTOR_MID_POSITION)
    target_output = int(target_output)

    if target_output > current_output + max_step:
        target_output = current_output + max_step
    elif target_output < current_output - max_step:
        target_output = current_output - max_step

    transition_limit = max(DRIVE_MAX_PULSE,
                           abs(TASK4_LAUNCH_COMPENSATION_PULSE),
                           abs(TASK4_STOP_COMPENSATION_PULSE))
    if target_output < -transition_limit:
        target_output = -transition_limit
    elif target_output > transition_limit:
        target_output = transition_limit

    absolute_position = int(MOTOR_MID_POSITION + target_output)
    if motor_position_output is None or absolute_position != motor_position_output:
        if control_motor_absolute_position(absolute_position, TILT_SPEED):
            motor_position_output = absolute_position

    return int(target_output)

def start_task(task_id, ball_valid, ball_position):
    global active_task, motion_state, target_x, stable_frame_count, previous_error, pd_initialized, last_cmd_time
    global balance_integral, balance_integral_output, balance_integral_counter
    global task4_compensation_start_time
    stop_motor()
    active_task = task_id
    stable_frame_count = 0
    balance_integral = 0.0
    balance_integral_output = 0.0
    balance_integral_counter = 0
    last_cmd_time = time.ticks_ms()

    if task_id == 3:
        motion_state = STATE_TASK3_TO_CENTER
        target_x = CENTER_X
        previous_error = (ball_position - target_x) if ball_valid else 0
        pd_initialized = ball_valid
        print("TASK 3 START")
    elif task_id == 4:
        motion_state = STATE_TASK4_LAUNCH_COMPENSATION
        target_x = CENTER_X
        previous_error = (ball_position - target_x) if ball_valid else 0
        pd_initialized = ball_valid
        task4_compensation_start_time = last_cmd_time
        print("TASK 4 LAUNCH FEEDFORWARD")
    elif task_id == 5:
        motion_state = STATE_TASK4_LAUNCH_COMPENSATION
        target_x = CENTER_X
        previous_error = (ball_position - target_x) if ball_valid else 0
        pd_initialized = ball_valid
        task4_compensation_start_time = last_cmd_time
        print("TASK 5 LAUNCH FEEDFORWARD")
    elif task_id == 6:
        task4_compensation_start_time = last_cmd_time
        if ball_valid:
            target_x = int(ball_position)
            motion_state = STATE_TASK4_LAUNCH_COMPENSATION
            previous_error = 0
            pd_initialized = True
            print("TASK 6 LAUNCH FEEDFORWARD, TARGET:", target_x)
        else:
            motion_state = STATE_TASK6_CAPTURE_POSITION
            target_x = CENTER_X
            previous_error = 0
            pd_initialized = False
            print("TASK 6 WAIT BALL")

def start_task4_stop_compensation(ball_valid, ball_position):
    global active_task, motion_state, target_x, stable_frame_count
    global previous_error, pd_initialized, last_cmd_time
    global balance_integral, balance_integral_output, balance_integral_counter
    global task4_compensation_start_time

    active_task = 4
    motion_state = STATE_TASK4_STOP_COMPENSATION
    target_x = CENTER_X
    stable_frame_count = 0
    previous_error = (ball_position - target_x) if ball_valid else 0
    pd_initialized = ball_valid
    balance_integral = 0.0
    balance_integral_output = 0.0
    balance_integral_counter = 0
    last_cmd_time = time.ticks_ms()
    task4_compensation_start_time = last_cmd_time
    initial_output = TASK4_STOP_COMPENSATION_PULSE
    if ball_valid:
        initial_output += balanceX(ball_position)
    set_motor_position(initial_output)
    print("TASK 4 STOP FEEDFORWARD")

def start_task5_decel_compensation():
    global active_task, motion_state, target_x
    global task4_compensation_start_time

    if active_task != 6:
        active_task = 5
        target_x = CENTER_X
    motion_state = STATE_TASK5_DECEL_COMPENSATION
    task4_compensation_start_time = time.ticks_ms()
    print("TASK", active_task, "DECEL FEEDFORWARD")

def start_task45_stop_compensation(ball_valid, ball_position):
    global motion_state, stable_frame_count
    global previous_error, pd_initialized, last_cmd_time
    global balance_integral, balance_integral_output, balance_integral_counter
    global task4_compensation_start_time

    motion_state = STATE_TASK4_STOP_COMPENSATION
    stable_frame_count = 0
    previous_error = (ball_position - target_x) if ball_valid else 0
    pd_initialized = ball_valid
    balance_integral = 0.0
    balance_integral_output = 0.0
    balance_integral_counter = 0
    last_cmd_time = time.ticks_ms()
    task4_compensation_start_time = last_cmd_time
    initial_output = TASK4_STOP_COMPENSATION_PULSE
    if ball_valid:
        initial_output += balanceX(ball_position)
    set_motor_position(initial_output)
    print("TASK", active_task, "STOP FEEDFORWARD")

def prepare_task6():
    global active_task, motion_state, target_x, stable_frame_count
    global previous_error, pd_initialized
    global balance_integral, balance_integral_output, balance_integral_counter

    active_task = 6
    motion_state = STATE_TASK6_ARMED
    target_x = CENTER_X
    stable_frame_count = 0
    previous_error = 0
    pd_initialized = False
    balance_integral = 0.0
    balance_integral_output = 0.0
    balance_integral_counter = 0
    set_motor_position(0)
    print("TASK 6 ARMED - PID PAUSED")

def cancel_task6_prepare(ball_valid, ball_position):
    global active_task, motion_state, target_x, stable_frame_count
    global previous_error, pd_initialized
    global balance_integral, balance_integral_output, balance_integral_counter

    active_task = 0
    motion_state = STATE_HOLD_CENTER
    target_x = CENTER_X
    stable_frame_count = 0
    previous_error = (ball_position - target_x) if ball_valid else 0
    pd_initialized = ball_valid
    balance_integral = 0.0
    balance_integral_output = 0.0
    balance_integral_counter = 0
    print("TASK 6 ARM CANCELLED")

def process_uart_commands(ball_valid, ball_position):
    global rx_state, rx_task_id
    if not uart or uart.any() <= 0: return
    received_data = uart.read()
    if not received_data: return

    for value in received_data:
        if rx_state == RX_WAIT_HEADER:
            if value == 0xFF: rx_state = RX_WAIT_TASK
        elif rx_state == RX_WAIT_TASK:
            if value in (0x03, 0x04, 0x05, 0x06,
                         0x14, 0x15, 0x16, 0x17):
                rx_task_id = value
                rx_state = RX_WAIT_END
            elif value == 0xFF: rx_state = RX_WAIT_TASK
            else: rx_state = RX_WAIT_HEADER
        elif rx_state == RX_WAIT_END:
            if value == 0x0D:
                if rx_task_id == 0x14:
                    start_task4_stop_compensation(ball_valid, ball_position)
                elif rx_task_id == 0x15:
                    start_task5_decel_compensation()
                elif rx_task_id == 0x18:
                    start_task45_stop_compensation(ball_valid, ball_position)
                elif rx_task_id == 0x16:
                    prepare_task6()
                elif rx_task_id == 0x17:
                    cancel_task6_prepare(ball_valid, ball_position)
                else:
                    start_task(rx_task_id, ball_valid, ball_position)
            rx_state = RX_WAIT_TASK if value == 0xFF else RX_WAIT_HEADER

# ============================================================
# 7. 核心控制子线程 (Vision + Control)
# ============================================================
def control_thread(sensor_obj):
    global latest_img, thread_running, last_cmd_time
    global motion_state, target_x, active_task, stable_frame_count, previous_error, pd_initialized
    global balance_integral, balance_integral_output, balance_integral_counter

    clock = time.clock()
    frame_count = 0
    last_cmd_time = time.ticks_ms()
    thread_running = True
    print("控制与视觉分析线程已启动")

    while thread_running:
        clock.tick()
        frame_count += 1
        img = sensor_obj.snapshot()
        if img is None or img == -1:
            time.sleep_ms(2)
            continue

        current_time = time.ticks_ms()
        ball_x, ball_valid = 0, False
        action_display = motor_position_output if motor_position_output is not None else 0
        status_str = "READY HOLD 0"

        # --- 图传分支：截取干净的图像传给网页 ---
        # 要求：不包含任何绘制字符或框线
        clean_stream_img = img.copy(roi=STREAM_ROI)
        with thread_lock:
            old_img = latest_img
            latest_img = clean_stream_img
        if old_img:
            del old_img

        # --- 本地视觉与UI绘制 ---
        img.draw_rectangle(ROI, color=255, thickness=1)
        blobs = img.find_blobs(BALL_THRESHOLDS, roi=ROI, invert=False, area_threshold=50, merge=True)

        # 对候选目标进行尺寸、像素面积和宽高比筛选
        valid_ball_blobs = []
        for blob in blobs:
            blob_w = blob.w()
            blob_h = blob.h()
            blob_pixels = blob.pixels()

            if blob_w < BALL_MIN_W or blob_w > BALL_MAX_W:
                continue
            if blob_h < BALL_MIN_H or blob_h > BALL_MAX_H:
                continue
            if blob_pixels < BALL_MIN_PIXELS or blob_pixels > BALL_MAX_PIXELS:
                continue

            aspect_ratio = blob_w / blob_h
            if aspect_ratio < BALL_MIN_ASPECT or aspect_ratio > BALL_MAX_ASPECT:
                continue

            valid_ball_blobs.append(blob)

        if valid_ball_blobs:
            largest_blob = max(valid_ball_blobs, key=lambda b: b.pixels())
            ball_x = largest_blob.cx()
            ball_valid = True
            img.draw_rectangle(largest_blob.rect(), color=255, thickness=2)
            img.draw_cross(ball_x, largest_blob.cy(), color=255, size=5, thickness=2)

        # --- 串口解析与状态机流转 ---
        process_uart_commands(ball_valid, ball_x)

        if motion_state == STATE_TASK6_CAPTURE_POSITION and ball_valid:
            target_x = int(ball_x)
            launch_elapsed = time.ticks_diff(
                current_time, task4_compensation_start_time)
            motion_state = (STATE_TASK4_LAUNCH_COMPENSATION
                            if launch_elapsed < TASK5_LAUNCH_COMPENSATION_MS
                            else STATE_TASK6_HOLD_POSITION)
            previous_error, pd_initialized, stable_frame_count = 0, True, 0

        if motion_state == STATE_IDLE:
            active_task, motion_state, target_x = 0, STATE_HOLD_CENTER, CENTER_X
            previous_error, pd_initialized, stable_frame_count = 0, False, 0

        # Match the feedforward to the acceleration of the STM32 smoothstep
        # speed ramp: zero at both ends and maximum halfway through.
        task4_launch_active = motion_state == STATE_TASK4_LAUNCH_COMPENSATION
        task4_stop_active = motion_state == STATE_TASK4_STOP_COMPENSATION
        task5_decel_active = motion_state == STATE_TASK5_DECEL_COMPENSATION
        task4_feedforward = 0
        if task4_launch_active:
            launch_pulse = (TASK5_LAUNCH_COMPENSATION_PULSE
                            if active_task in (5, 6)
                            else TASK4_LAUNCH_COMPENSATION_PULSE)
            launch_duration_ms = (TASK5_LAUNCH_COMPENSATION_MS
                                  if active_task in (5, 6)
                                  else TASK4_LAUNCH_COMPENSATION_MS)
            launch_elapsed = time.ticks_diff(
                current_time, task4_compensation_start_time)
            if launch_elapsed >= launch_duration_ms:
                motion_state = (STATE_TASK6_HOLD_POSITION
                                if active_task == 6
                                else STATE_HOLD_CENTER)
                task4_launch_active = False
            else:
                launch_progress = (
                    launch_elapsed * 1000
                ) // launch_duration_ms
                task4_feedforward = (
                    launch_pulse * 4 *
                    launch_progress * (1000 - launch_progress)
                ) // 1000000
        elif task4_stop_active:
            stop_elapsed = time.ticks_diff(
                current_time, task4_compensation_start_time)
            if stop_elapsed >= TASK4_STOP_COMPENSATION_MS:
                motion_state = (STATE_TASK6_HOLD_POSITION
                                if active_task == 6
                                else STATE_HOLD_CENTER)
                task4_stop_active = False
            else:
                stop_progress = (
                    stop_elapsed * 1000
                ) // TASK4_STOP_COMPENSATION_MS
                stop_decay = 1000 - (
                    stop_progress * stop_progress *
                    (3000 - 2 * stop_progress)
                ) // 1000000
                stop_magnitude = (
                    abs(TASK4_STOP_COMPENSATION_PULSE) * stop_decay
                ) // 1000
                task4_feedforward = (-stop_magnitude
                                     if TASK4_STOP_COMPENSATION_PULSE < 0
                                     else stop_magnitude)
        elif task5_decel_active:
            decel_elapsed = time.ticks_diff(
                current_time, task4_compensation_start_time)
            if decel_elapsed >= TASK5_DECEL_COMPENSATION_MS:
                motion_state = (STATE_TASK6_HOLD_POSITION
                                if active_task == 6
                                else STATE_HOLD_CENTER)
                task5_decel_active = False
            else:
                decel_progress = (
                    decel_elapsed * 1000
                ) // TASK5_DECEL_COMPENSATION_MS
                task4_feedforward = (
                    TASK5_DECEL_COMPENSATION_PULSE * 4 *
                    decel_progress * (1000 - decel_progress)
                ) // 1000000

        task4_feedforward_active = (task4_launch_active or
                                    task4_stop_active or
                                    task5_decel_active)

        # --- 按板球系统结构进行单轴PID位置控制 ---
        if motion_state == STATE_TASK6_ARMED:
            action_display = set_motor_position(0)
        elif ball_valid:
            error = ball_x - target_x
            if motion_state == STATE_TASK3_TO_POSITIVE:
                stable_tolerance_px = TASK3_POSITIVE_TOLERANCE_PX
            elif active_task == 3:
                stable_tolerance_px = TASK3_FINAL_TOLERANCE_PX
            else:
                stable_tolerance_px = DEADZONE
            task3_negative_settled = not (
                motion_state == STATE_TASK3_TO_NEGATIVE and
                abs(error - previous_error) > TASK3_STATIONARY_DELTA_PX)
            if (abs(error) <= stable_tolerance_px and
                    task3_negative_settled):
                stable_frame_count += 1
                if stable_frame_count >= TARGET_STABLE_FRAMES:
                    if motion_state == STATE_TASK3_TO_CENTER:
                        motion_state, target_x = STATE_TASK3_TO_POSITIVE, POSITIVE_TARGET_X
                        previous_error, pd_initialized, stable_frame_count = ball_x - target_x, True, 0
                    elif motion_state == STATE_TASK3_TO_POSITIVE:
                        motion_state, target_x = STATE_TASK3_TO_NEGATIVE, NEGATIVE_TARGET_X
                        previous_error, pd_initialized, stable_frame_count = ball_x - target_x, True, 0
                    elif motion_state == STATE_TASK3_TO_NEGATIVE:
                        motion_state, target_x = STATE_TASK3_HOLD_NEGATIVE, NEGATIVE_TARGET_X
                        previous_error, pd_initialized, stable_frame_count = ball_x - target_x, True, 0
                    else:
                        stable_frame_count = TARGET_STABLE_FRAMES
            else:
                stable_frame_count = 0

            # 与板球TIM4第四时间片一致，每50ms计算一次控制输出。
            # 即使进入目标死区也继续计算，使管道能够回到零倾角，
            # 而不是保持上一次的倾斜位置。
            if time.ticks_diff(current_time, last_cmd_time) >= CONTROL_INTERVAL:
                target_output = balanceX(ball_x)
                if task4_feedforward_active:
                    target_output += task4_feedforward
                action_display = set_motor_position(target_output)
                last_cmd_time = current_time
        elif task4_feedforward_active:
            status_str = "LOST FF"
            previous_error, pd_initialized, stable_frame_count = 0, False, 0
            balance_integral = 0.0
            balance_integral_output = 0.0
            balance_integral_counter = 0
            if time.ticks_diff(current_time, last_cmd_time) >= CONTROL_INTERVAL:
                action_display = set_motor_position(task4_feedforward)
                last_cmd_time = current_time
        else:
            status_str = "LOST"
            previous_error, pd_initialized, stable_frame_count = 0, False, 0
            balance_integral = 0.0
            balance_integral_output = 0.0
            balance_integral_counter = 0
            if time.ticks_diff(current_time, last_cmd_time) > BALL_LOST_STOP_TIME:
                stop_motor()
                set_motor_position(0)
                last_cmd_time = current_time

        # --- 更新状态文本 ---
        if motion_state == STATE_TASK3_TO_CENTER: status_str = "T3 CHECK 0"
        elif motion_state == STATE_TASK3_TO_POSITIVE: status_str = "T3 TO +5CM"
        elif motion_state == STATE_TASK3_TO_NEGATIVE: status_str = "T3 TO -5CM"
        elif motion_state == STATE_TASK3_HOLD_NEGATIVE: status_str = "T3 HOLD -5"
        elif motion_state == STATE_TASK4_LAUNCH_COMPENSATION:
            status_str = ("T6 LAUNCH FF50" if active_task == 6
                          else ("T5 LAUNCH FF50" if active_task == 5
                                else "T4 LAUNCH FF50"))
        elif motion_state == STATE_TASK4_STOP_COMPENSATION:
            status_str = ("T6 STOP FF-20" if active_task == 6
                          else ("T5 STOP FF-20" if active_task == 5
                                else "T4 STOP FF-20"))
        elif motion_state == STATE_TASK4_COMPENSATION_RELEASE: status_str = "T4 PID RAMP"
        elif motion_state == STATE_TASK5_DECEL_COMPENSATION:
            status_str = ("T6 DECEL FF-20" if active_task == 6
                          else "T5 DECEL FF-20")
        elif motion_state == STATE_HOLD_CENTER: status_str = "READY HOLD 0" if active_task == 0 else "T%d HOLD 0" % active_task
        elif motion_state == STATE_TASK6_CAPTURE_POSITION: status_str = "T6 CAPTURE"
        elif motion_state == STATE_TASK6_HOLD_POSITION: status_str = "T6 HOLD"
        elif motion_state == STATE_TASK6_ARMED: status_str = "T6 ARMED P0"

        # --- 本地UI显示绘制 (这些不会被传到网页) ---
        img.draw_line(CENTER_X, 0, CENTER_X, IMG_H, color=120, thickness=1)
        img.draw_line(POSITIVE_TARGET_X, 0, POSITIVE_TARGET_X, IMG_H, color=100, thickness=1)
        img.draw_line(NEGATIVE_TARGET_X, 0, NEGATIVE_TARGET_X, IMG_H, color=100, thickness=1)
        img.draw_line(int(target_x), 0, int(target_x), IMG_H, color=255, thickness=2)

        info_text = ('[%s] X:%d T:%d P:%d' % (status_str, ball_x, target_x, action_display))
        img.draw_string_advanced(8, 8, 22, info_text, color=255)

        Display.show_image(img, x=(LCD_W - IMG_W) // 2, y=(LCD_H - IMG_H) // 2)

        if frame_count % 30 == 0: gc.collect()

# ============================================================
# 8. Web 服务与图传函数 (运行在主线程)
# ============================================================
def start_wifi_ap():
    ap = network.WLAN(network.AP_IF)
    ap.active(False)
    time.sleep(1)
    ap.active(True)
    ap.config(ssid=AP_SSID, key=AP_KEY)
    time.sleep(2)
    net_info = ap.ifconfig()
    print("========================================")
    print("Wi-Fi AP started | SSID:", AP_SSID)
    print("WEB URL: http://%s:%d/" % (net_info[0], HTTP_PORT))
    print("========================================")
    return ap, net_info[0]

def send_all(sock, data, timeout_ms=5000):
    total, sent_total = len(data), 0
    start_ms = time.ticks_ms()
    while sent_total < total:
        try:
            sent = sock.send(data[sent_total:])
            if sent and sent > 0:
                sent_total += sent
                start_ms = time.ticks_ms()
        except OSError as err:
            if err.args and err.args[0] in (11,): pass
            else: raise
        if time.ticks_diff(time.ticks_ms(), start_ms) > timeout_ms: raise OSError("socket timeout")
        time.sleep_ms(2)
    return sent_total

def stream_mjpeg(client):
    header = ("HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=%s\r\n\r\n" % STREAM_BOUNDARY)
    try: client.setblocking(False)
    except: pass
    send_all(client, header.encode("utf-8"))

    frame_interval_ms = 1000 // TARGET_FPS
    next_frame_ms = time.ticks_ms()
    while True:
        now_ms = time.ticks_ms()
        wait_ms = time.ticks_diff(next_frame_ms, now_ms)
        if wait_ms > 0: time.sleep_ms(wait_ms)

        frame_start_ms = time.ticks_ms()
        with thread_lock:
            img = latest_img
            if img is None:
                time.sleep_ms(5)
                continue
            # 压缩灰度图 (不带UI的图传专属Frame)
            jpeg = img.compressed(quality=JPEG_QUALITY)

        jpeg_data = jpeg.bytearray()
        part_header = ("--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % (STREAM_BOUNDARY, len(jpeg_data)))
        try:
            send_all(client, part_header.encode("utf-8"))
            send_all(client, memoryview(jpeg_data))
            send_all(client, b"\r\n")
        except:
            break
        next_frame_ms = time.ticks_add(frame_start_ms, frame_interval_ms)

def send_index_page(client, ap_ip):
    # 保持原来的前端页面及录像功能，仅将尺寸改为 800x128
    html = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>K230 凹槽实时画面</title>
<style>
body { background: #111; color: #eee; font-family: Arial; text-align: center; }
img, video { width: 100%%; max-width: 800px; height: auto; border: 1px solid #555; background: #000; }
#recordCanvas { display: none; }
#playbackVideo { display: none; }
button { padding:10px 16px; margin:4px; border:none; border-radius:4px; font-size:15px; }
#startRecord {background:#d03030;color:#fff;}
#stopRecord {background:#555;color:#fff;}
#playRecord {background:#208844;color:#fff;}
button:disabled {opacity:0.4;}
.downloads a { display:block; color:#6cf; margin:6px 0; }
</style>
</head><body>
<h2>K230 凹槽实时画面</h2>
<p style="color:#bbb;">实时灰度图传 (纯净流)</p>
<img id="streamImage" src="/stream">
<canvas id="recordCanvas" width="%d" height="%d"></canvas>
<video id="playbackVideo" controls playsinline></video>
<div>
    <button id="startRecord">开始录像</button>
    <button id="stopRecord" disabled>停止</button>
    <button id="playRecord" disabled>回放</button>
</div>
<p id="recordStatus"></p>
<div class="downloads"><div id="downloads"></div></div>
<script>
(function(){
    const img = document.getElementById('streamImage'), cvs = document.getElementById('recordCanvas'), ctx = cvs.getContext('2d');
    const vid = document.getElementById('playbackVideo'), btnStart = document.getElementById('startRecord');
    const btnStop = document.getElementById('stopRecord'), btnPlay = document.getElementById('playRecord');
    const status = document.getElementById('recordStatus'), dl = document.getElementById('downloads');
    let rec = null, chunks = [], url = null;

    function draw(){
        if(img.complete && img.naturalWidth>0) try{ ctx.drawImage(img,0,0,cvs.width,cvs.height); }catch(e){}
        requestAnimationFrame(draw);
    }
    draw();

    btnStart.onclick = function(){
        img.style.display='block'; vid.style.display='none';
        try {
            const stream = cvs.captureStream(15);
            rec = new MediaRecorder(stream);
            chunks = [];
            rec.ondataavailable = e => { if(e.data.size>0) chunks.push(e.data); };
            rec.onstop = () => {
                const blob = new Blob(chunks, {type: 'video/webm'});
                url = URL.createObjectURL(blob);
                vid.src = url;
                const a = document.createElement('a');
                a.href = url; a.download = 'record.webm'; a.textContent = '下载最新录像';
                dl.prepend(a);
                status.textContent = '录像已停止';
            };
            rec.start(1000);
            btnStart.disabled=true; btnStop.disabled=false; btnPlay.disabled=true;
            status.textContent = '正在录像...';
        } catch(e) { status.textContent = '浏览器不支持'; }
    };
    btnStop.onclick = function(){
        if(rec) rec.stop();
        btnStart.disabled=false; btnStop.disabled=true; btnPlay.disabled=false;
    };
    btnPlay.onclick = function(){
        img.style.display='none'; vid.style.display='block';
        vid.currentTime=0; vid.play();
        status.textContent = '正在回放...';
    };
})();
</script>
</body></html>""" % (STREAM_W, STREAM_H)
    header = ("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: %d\r\n\r\n" % len(html.encode("utf-8")))
    send_all(client, header.encode("utf-8"))
    send_all(client, html.encode("utf-8"))
    time.sleep_ms(120)

def run_http_server(ap_ip):
    addr = socket.getaddrinfo(ap_ip, HTTP_PORT, socket.AF_INET, socket.SOCK_STREAM)[0][-1]
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(addr)
    server.listen(3)
    server.setblocking(False)
    print("HTTP Server listening on", addr)

    while True:
        try:
            client, _ = server.accept()
            request = b""
            start_ms = time.ticks_ms()
            client.setblocking(False)
            while time.ticks_diff(time.ticks_ms(), start_ms) < 3000:
                try:
                    chunk = client.recv(512)
                    if not chunk: break
                    request += chunk
                    if b"\r\n\r\n" in request: break
                except OSError as e:
                    if e.args[0] == 11: time.sleep_ms(5)

            if not request:
                client.close()
                continue

            path = request.split(b"\r\n")[0].split(b" ")[1].decode()
            if path == "/": send_index_page(client, ap_ip)
            elif path.startswith("/stream"): stream_mjpeg(client)
            client.close()
        except OSError as e:
            if e.args and e.args[0] in (11, 103): time.sleep_ms(20)

# ============================================================
# 9. 系统入口
# ============================================================
if __name__ == "__main__":
    try:
        # 1. 硬件初始化
        fpioa = FPIOA()
        fpioa.set_function(3, FPIOA.UART1_TXD)
        fpioa.set_function(4, FPIOA.UART1_RXD)
        uart = UART(UART.UART1, 115200)

        sensor = Sensor(id=2, width=1280, height=960, fps=90)
        sensor.reset()
        sensor.set_framesize(width=IMG_W, height=IMG_H)
        sensor.set_pixformat(Sensor.GRAYSCALE)

        Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=True, quality=100)
        MediaManager.init()
        sensor.run()
        time.sleep(1)

        # 2. 启动Wi-Fi AP
        ap, ap_ip = start_wifi_ap()

        # 3. 启动后台控制与视觉检测子线程
        _thread.start_new_thread(control_thread, (sensor,))
        time.sleep(1)

        # 4. 主线程运行HTTP服务器
        run_http_server(ap_ip)

    except KeyboardInterrupt:
        print("手动停止")
    finally:
        thread_running = False
        time.sleep(1)
        stop_motor()
        try: sensor.stop()
        except: pass
        try: Display.deinit()
        except: pass
        try: MediaManager.deinit()
        except: pass
        gc.collect()
