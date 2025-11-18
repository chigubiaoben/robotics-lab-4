#!/usr/bin/env pybricks-micropython
# 左转没问题（已加入“锐角左转≈120°”处理）
from pybricks.ev3brick import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction, Stop, Button
from pybricks.tools import wait, StopWatch
from math import pi, sqrt, cos, sin, radians
from math import atan2, degrees

# ---------- Hardware ----------
ev3 = EV3Brick()
LEFT  = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
RIGHT = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
GYRO  = GyroSensor(Port.S1)
front_BUMP = TouchSensor(Port.S4)
Ultrasonic = UltrasonicSensor(Port.S2)

# ---------- Geometry / Units ----------
WHEEL_DIAM_MM   = 56
TRACK_WIDTH_MM  = 118
MM_PER_DEG      = (pi * WHEEL_DIAM_MM) / 360.0

# ---------- Control Gains ----------
Kp_h, Kd_h = 4.0, 0.04          # heading PD
Kp_d, Kd_d = 0.007, 0.002       # wall PD
EMA_ALPHA  = 0.25
MAX_D_STEP = 30
MAX_DIFF   = 250

# ---------- Speeds ----------
SPEED_MM_S        = 220         # initial approach
FOLLOW_SPEED_MM_S = 160         # wall-follow
MAX_SPEED         = 400
MIN_SPEED         = 120
MAX_SPEED_FOLLOW  = 340
MIN_SPEED_FOLLOW  = 90

# ---------- Mission Parameters ----------
TARGET_DIST_MM   = 135          # desired left clearance
PERIM_MIN_MM     = 1500         # min perimeter before loop-close check
HIT_RADIUS_MM    = 100          # near-hitpoint window to stop following
HIT_BACKOFF_MM   = 140.0        # backoff after first bump to set hitpoint
BUMP_BACKOFF_MM  = 100.0        # backoff on bump while following
BUMP_RIGHT_DEG   = 45.0         # small right turn after follow-bump
BUMP_SURGE_MM    = 120.0

# Corner recovery (inline values used below; feel free to tweak)
CORNER_PEEK_RIGHT_DEG = 30.0    # yaw right to see next wall face
CORNER_SURGE_MM       = 120.0   # forward surge distance after peek
ULTRA_TAPE_GLITCH     = 337     # keep this for dark tape artifact
ULTRA_MAX_RANGE       = 2500    # treat >= this as "no wall" (max ~2550 mm)

# --- Sharp LEFT-corner handling (120°-class bends) ---
LEFT_CORNER_GAP      = 90.0     # mm: d_s must exceed target by this much
LEFT_CORNER_DE_DOT   = 700.0    # mm/s: d_s must be increasing at least this fast
CORNER_PEEK_LEFT_DEG = 60.0     # deg: in-place left pivot to "see" new wall face
CORNER_SURGE_LEFT_MM = 160.0    # mm: surge forward to re-engage the wall



#坐标
x = None
y = None

goal_x = 2500
goal_y = 2500

x_entry = None    # 进入 wall-follow 时的位置
y_entry = None

dist_entry_to_goal = None  # entry point 到目标的距离

_last_deg_avg = None
base_heading = None

#Timer for wall-follow
timer = StopWatch()



#把当前坐标显示在屏幕上
def print_xy():
    ev3.screen.clear()
    ev3.screen.print("x = {:.0f} mm".format(x))
    ev3.screen.print("y = {:.0f} mm".format(y))

# ---------- Helpers ----------
#把线速度（mm/s）转换成轮子转速（度/秒）
def mps_to_dps(v_mm_s):
    return (v_mm_s / (pi * WHEEL_DIAM_MM)) * 360.0

#限制最大、最小速度
def set_wheels(vL, vR, vmax, vmin):
    vL = max(-vmax, min(vmax, vL))
    vR = max(-vmax, min(vmax, vR))
    if 0 < abs(vL) < vmin: vL = vmin if vL > 0 else -vmin
    if 0 < abs(vR) < vmin: vR = vmin if vR > 0 else -vmin
    LEFT.run(vL); RIGHT.run(vR)

#读出陀螺仪当前累计角度（度）
def gyro_zero(): 
    return GYRO.angle()

#得到相对于base的角度变化（也就是相对角度）
def gyro_angle_rel(base): 
    return GYRO.angle() - base

#让机器人在原地转向，直到朝向达到目标角度
def turn_to_heading_with_base(target_deg, base, tol=2.0, kp=3.0, max_sp=200):
    while True:
        e = target_deg - gyro_angle_rel(base)
        if abs(e) < tol: break
        turn = max(-max_sp, min(max_sp, kp * e))
        LEFT.run( turn); RIGHT.run(-turn)
        wait(10)
    LEFT.stop(Stop.HOLD); RIGHT.stop(Stop.HOLD)

#让机器人沿指定 heading 直走
def drive_straight_heading_with_base(target_deg, base, speed_mm_s, stop_cond_fn, max_time_ms=300000):
    v_cmd = mps_to_dps(speed_mm_s)
    timer = StopWatch()
    while timer.time() < max_time_ms:
        theta = gyro_angle_rel(base) - target_deg
        omega = GYRO.speed()
        head_cmd = Kp_h * (-theta) - Kd_h * omega
        headroom = MAX_SPEED - v_cmd
        head_cmd = max(-headroom, min(headroom, head_cmd))
        vL = v_cmd - head_cmd; vR = v_cmd + head_cmd
        set_wheels(vL, vR, MAX_SPEED, MIN_SPEED)
        if stop_cond_fn(): break
        wait(20)
    LEFT.stop(Stop.BRAKE); RIGHT.stop(Stop.BRAKE)

# ---------- Odometry (origin at hit point after backoff+right turn) ----------


#dead reckoning计算坐标
def odom_update():
    """Integrate translation only; in-place turns give ds≈0 (avg wheel motion)."""
    global x, y, _last_deg_avg
    deg_avg = 0.5 * (LEFT.angle() + RIGHT.angle())
    ddeg = deg_avg - _last_deg_avg
    _last_deg_avg = deg_avg
    ds = ddeg * MM_PER_DEG
    th = radians(gyro_angle_rel(base_heading))
    x += ds * cos(th); y += ds * sin(th)
    return ds

#原地转圈
def odom_freeze_wheels():
    """Reset odom encoder baseline without changing x,y (use after in-place turns)."""
    global _last_deg_avg
    _last_deg_avg = 0.5 * (LEFT.angle() + RIGHT.angle())

#计算两个坐标点之间的距离
def dist_mm(ax, ay, bx, by):
    dx, dy = (ax - bx), (ay - by)
    return sqrt(dx*dx + dy*dy)

#简化版turn_to_heading_with_base
def turn_to_heading(target_deg, tol=2.0, kp=3.0, max_sp=200):
    turn_to_heading_with_base(target_deg, base_heading, tol, kp, max_sp)

#简化版drive_straight_heading_with_base
def drive_straight_heading(target_deg, speed_mm_s, stop_cond_fn, max_time_ms=300000):
    drive_straight_heading_with_base(target_deg, base_heading, speed_mm_s, stop_cond_fn, max_time_ms)




#goal (2.5, 2.5)
#start (0.5, 0)

#限时3 min
#用bug 2 algorithm
#每绕过一个obstacle的时候记录距离goal最近的坐标，然后回到那个坐标，继续前往goal
#接触obstacle：先bump，然后后退，然后右转绕行



def heading_to_goal():
    dx = goal_x - x
    dy = goal_y - y
    return degrees(atan2(dy, dx))

def odom_reset_wheels():
    global _last_deg_avg
    LEFT.reset_angle(0)
    RIGHT.reset_angle(0)
    _last_deg_avg = 0.0


def init_bug2_odometry():
    global base_heading, x, y, _last_deg_avg
    x = 500   # mm
    y = 0     # mm
    base_heading = gyro_zero()  #只更新这一次！
    LEFT.reset_angle(0)
    RIGHT.reset_angle(0)
    _last_deg_avg = 0.0


# m-line判断
def on_m_line(x, y, tol=50):
    # 点到直线 (x - y - 500 = 0) 的距离 < tol
    dist = abs(x - y - 500) / 1.414  # divide by sqrt(2)
    return dist < tol



#goal region判断
def in_goal_region(x, y):
    return dist_mm(x, y, goal_x, goal_y) < 300
# def in_goal_region(x, y, tol_mm=300):
#     dx = x - goal_x
#     dy = y - goal_y
#     return dx*dx + dy*dy <= tol_mm * tol_mm


def bump_backoff_and_align():
    # 1. 后退
    backoff_deg = (HIT_BACKOFF_MM / (pi * WHEEL_DIAM_MM)) * 360.0
    LEFT.run_angle(speed=240, rotation_angle=-backoff_deg, then=Stop.HOLD, wait=False)
    RIGHT.run_angle(speed=240, rotation_angle=-backoff_deg, then=Stop.HOLD, wait=True)
    odom_update()

    # 2. 右转 (using global base_heading, do NOT reset gyro)
    curr = gyro_angle_rel(base_heading)
    turn_to_heading(curr + 90.0)

    # 3. Reset wheel baseline (but keep world x,y unchanged)
    odom_reset_wheels()


def wall_follow_step():

    global d_s, d_prev, e_prev, last_time_ms, x, y

    loop_t = timer.time()   # step 开始时间

    # --- 计算 dt ---
    now_ms = timer.time()
    dt_s = max(0.01, (now_ms - last_time_ms) / 1000.0)
    last_time_ms = now_ms

    # ---------------------------------------------------------
    # (A) bump safety — 撞墙时的恢复处理
    # ---------------------------------------------------------
    if front_BUMP.pressed():
        LEFT.stop(Stop.BRAKE); RIGHT.stop(Stop.BRAKE)

        # 后退
        backoff_deg = (BUMP_BACKOFF_MM / (pi * WHEEL_DIAM_MM)) * 360.0
        LEFT.run_angle(240, -backoff_deg, then=Stop.HOLD, wait=False)
        RIGHT.run_angle(240, -backoff_deg, then=Stop.HOLD, wait=True)
        odom_update()

        # 小右转
        curr = gyro_angle_rel(base_heading)
        turn_to_heading(curr + BUMP_RIGHT_DEG)
        odom_reset_wheels()   # 不能用 odom_reset()

        # surge 一点
        surge_deg = (BUMP_SURGE_MM / (pi * WHEEL_DIAM_MM)) * 360.0
        LEFT.run_angle(240, surge_deg, then=Stop.HOLD, wait=False)
        RIGHT.run_angle(240, surge_deg, then=Stop.HOLD, wait=True)
        odom_update()

        return   # 完成一个 step

    # ---------------------------------------------------------
    # (B) ultrasonic read + smoothing + corner handling
    # ---------------------------------------------------------

    d_raw = Ultrasonic.distance()

    # Ultrasonic 最大范围 → peek right
    if d_raw is not None and d_raw >= ULTRA_MAX_RANGE:
        curr = gyro_angle_rel(base_heading)
        peek_dir = curr - CORNER_PEEK_RIGHT_DEG
        turn_to_heading(peek_dir)
        odom_reset_wheels()

        surge_deg = 360.0 * CORNER_SURGE_MM / (pi * WHEEL_DIAM_MM)
        LEFT.run_angle(240, surge_deg, then=Stop.HOLD, wait=False)
        RIGHT.run_angle(240, surge_deg, then=Stop.HOLD, wait=True)
        odom_update()

        d_raw = Ultrasonic.distance() or d_s

    # tape glitch
    if d_raw is None or (ULTRA_TAPE_GLITCH < d_raw < ULTRA_MAX_RANGE):
        d_raw = d_s + MAX_D_STEP

    # 限幅
    if abs(d_raw - d_s) > MAX_D_STEP:
        d_raw = d_s + (MAX_D_STEP if d_raw > d_s else -MAX_D_STEP)

    # EMA 平滑
    d_s = (1 - EMA_ALPHA) * d_s + EMA_ALPHA * d_raw

    # ---------------------------------------------------------
    # (B2) sharp left corner detect
    # ---------------------------------------------------------

    e_prov  = d_s - TARGET_DIST_MM
    de_prov = (d_s - d_prev) / 0.02  # 20ms

    if (e_prov > LEFT_CORNER_GAP) and (de_prov > LEFT_CORNER_DE_DOT):
        curr = gyro_angle_rel(base_heading)
        turn_to_heading(curr + CORNER_PEEK_LEFT_DEG)
        odom_reset_wheels()

        surge_deg = 360.0 * CORNER_SURGE_LEFT_MM / (pi * WHEEL_DIAM_MM)
        LEFT.run_angle(240, surge_deg, then=Stop.HOLD, wait=False)
        RIGHT.run_angle(240, surge_deg, then=Stop.HOLD, wait=True)
        odom_update()

        d_prev = d_s
        return

    # ---------------------------------------------------------
    # (C) wall-follow PD + gyro damping
    # ---------------------------------------------------------

    e  = d_s - TARGET_DIST_MM
    de = (e - e_prev) / 0.02
    e_prev = e

    steer_d = Kp_d * e + Kd_d * de
    omega = GYRO.speed()
    steer_h = -Kd_h * omega

    diff = max(-MAX_DIFF, min(MAX_DIFF, (steer_d * 180.0) + steer_h))

    vL = v_cmd_follow - diff
    vR = v_cmd_follow + diff

    set_wheels(vL, vR, MAX_SPEED_FOLLOW, MIN_SPEED_FOLLOW)

    # ---------------------------------------------------------
    # (D) odometry update
    # ---------------------------------------------------------
    odom_update()

    # ---------------------------------------------------------
    # 保持 20ms 控制周期
    # ---------------------------------------------------------
    spent = timer.time() - loop_t
    wait(max(1, 20 - spent))





def follow_wall_until_leave_point(x_entry, y_entry, dist_entry_to_goal):
    
    #进入wall following，记录x_entry和y_entry
    #while（不能离开障碍物）
        #每隔一段时间判断是否在m-line
        #如果在m-line，那么比较此时的坐标和entry坐标，如果与终点距离变小,那么说明是leave point
        #否则继续绕行
    
    global x, y

    global d_s, d_prev, e_prev, last_time_ms, v_cmd_follow
    d_s = Ultrasonic.distance() or TARGET_DIST_MM
    d_prev = d_s
    e_prev = 0
    v_cmd_follow = mps_to_dps(FOLLOW_SPEED_MM_S)
    last_time_ms = timer.time()

    while True:

        # 执行一步 wall-following
        wall_follow_step()

        # 检查是否到达目标区域
        if in_goal_region(x, y):
            return "goal"

        # 检查 Bug2 是否可以离开障碍物
        if on_m_line(x, y) and dist_mm(x, y, goal_x, goal_y) < dist_entry_to_goal:
            return "leave"
        
        # （可选）检测绕了一圈，障碍阻挡目标
        if dist_mm(x, y, x_entry, y_entry) < 80:
            return "loop"



def stop_and_beep():
    LEFT.stop(Stop.HOLD)
    RIGHT.stop(Stop.HOLD)
    ev3.speaker.beep()



def main():

    ev3.screen.print("Press CENTER to start")
    while Button.CENTER not in ev3.buttons.pressed():
        wait(10)

    init_bug2_odometry()

    #while（没有到达终点）
    while not in_goal_region(x, y):
        #朝向m-line（如何才能转到朝向m-line的方向？）
        angle = heading_to_goal()
        turn_to_heading(angle)

        odom_reset_wheels()
        #走m-line
        #直到与障碍物bump（这里bump如何才能确认可以撞到，如果不是撞一个平面的话）
        drive_straight_heading(target_deg=angle,
        speed_mm_s=SPEED_MM_S,
        stop_cond_fn=lambda: front_BUMP.pressed(),
        max_time_ms=120000)
        #后退
        bump_backoff_and_align()

        #记录与m-line相撞的坐标
        x_entry = x
        y_entry = y
        dist_entry_to_goal = dist_mm(x_entry, y_entry, goal_x, goal_y)

        #进行wall following直到可以离开障碍物
        leave_reason =  follow_wall_until_leave_point(x_entry, y_entry, dist_entry_to_goal)

        if leave_reason == "goal":
            #到达目标点
            stop_and_beep()
            return

        elif leave_reason == "leave":
            # 回到 m-line，继续沿 m-line 走
            continue

        elif leave_reason == "loop":
            # 目标被障碍包围（理论上不应该发生）
            ev3.screen.print("Goal unreachable")



#执行main函数           
main()

    
    




