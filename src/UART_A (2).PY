import sensor
import time
import image
from pyb import UART, LED
from math import pi, atan, degrees, sqrt

# ========== 参数配置区 ==========

# ----- 颜色阈值（需要实际校准） -----
# 红色交通标志：PANTONE 1795 C
RED_THRESHOLD = (11, 39, 7, 63, 6, 83)

# 绿色交通标志：PANTONE 802 C
GREEN_THRESHOLD = (15, 36, -69, -5, -40, 32)

# 橙色线条：规则13.9 CMYK(0,60,100,0) → RGB(255,102,0)
ORANGE_THRESHOLD = (21, 42, 5, 52, 13, 62)

# 蓝色线条：规则13.9 CMYK(100,80,0,0) → RGB(0,51,255)
BLUE_THRESHOLD = (20, 60, 10, 50, -80, -30)

# ----- 交通标志参数 -----
SIGN_WIDTH_CM = 5.0
SIGN_HEIGHT_CM = 10.0
F_PX = 558.0          # 像素焦距（标定值）
MIN_BLOB_AREA = 500
MAX_BLOB_AREA = 150000
MIN_CIRCULARITY = 0.1
MAX_CIRCULARITY = 0.98
MIN_ASPECT_RATIO = 0.2
MAX_ASPECT_RATIO = 1

# ----- 线条检测参数 -----
LINE_MIN_AREA = 50     # 线条色块最小面积（过滤噪声）

# ----- 摄像头参数 -----
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FOV_H = 70.0
ROI_RATIO = 0.33       # 交通标志ROI

# ----- UART 通信配置 -----
UART_BAUDRATE = 115200
UART_TIMEOUT = 1000

# ========== 全局状态 ==========
current_mode = 'I'  # I = Idle（待命）
last_mode = 'I'
mode_updated = False

# ========== 系统初始化 ==========
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_auto_whitebal(True)
sensor.set_auto_gain(True)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(60)

# 初始化串口（UART3: P4=TX, P5=RX）
uart = UART(3, UART_BAUDRATE, timeout_char=UART_TIMEOUT)

# LED指示灯
led_red = LED(1)
led_green = LED(2)
led_blue = LED(3)

frame_count = 0

# ========== 工具函数 ==========

def send_result(result_char, area=0):
    """
    向 ESP32 发送识别结果
    - 无检测：发送 'N'
    - 有检测（橙色/蓝色）：发送 'O:12345' 或 'B:12345'
    """
    if result_char == 'N' or result_char == 'I':
        uart.write(result_char + '\r\n')
        print("Sent:", result_char)
    elif result_char == 'R' or result_char == 'G':
        # 障碍赛结果由 send_obstacle_data 处理
        pass
    else:
        # 橙色或蓝色：发送 "O:12345" 或 "B:12345"
        msg = "{}{}\r\n".format(result_char, area)
        uart.write(msg)
        print("Sent:", msg.strip())

def send_obstacle_data(blobs, color_char):
    """
    只发送最近的一个标志（面积最大 = 距离最近）
    格式: COLOR,DIST_CM,ANGLE_DEG
    例如: R,45.3,-5.2
    """
    if not blobs:
        return

    # 按面积从大到小排序（面积越大 = 越近）
    blobs_sorted = sorted(blobs, key=lambda b: b.area(), reverse=True)
    best_blob = blobs_sorted[0]

    dist, angle, direction = estimate_distance_and_angle(best_blob)
    if dist > 0:
        msg = "{},{:.1f},{:.1f}\r\n".format(color_char, dist, angle)
        uart.write(msg)
        print("Sent (nearest):", msg.strip())

def is_valid_blob(blob):
    """判断色块是否为有效的交通标志"""
    area = blob.area()
    if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA:
        return False
    if blob.perimeter() > 0:
        circularity = (4 * pi * area) / (blob.perimeter() * blob.perimeter())
        if circularity < MIN_CIRCULARITY or circularity > MAX_CIRCULARITY:
            return False
    else:
        return False
    w = blob.w()
    h = blob.h()
    if h > 0:
        aspect_ratio = w / h
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            return False
    else:
        return False
    return True

def filter_blobs_by_roi(blobs, img_width, img_height, roi_ratio):
    """只保留在 ROI 区域内的色块"""
    roi_y_start = int(img_height * roi_ratio)
    filtered = []
    for blob in blobs:
        if blob.cy() >= roi_y_start:
            filtered.append(blob)
    return filtered

def estimate_distance_and_angle(blob):
    """估算交通标志的距离和偏移角度"""
    w_px = blob.w()
    h_px = blob.h()
    area_px = blob.area()

    # 高度法
    if h_px > 5:
        dist_height = (SIGN_HEIGHT_CM * F_PX) / h_px
    else:
        dist_height = -1

    # 宽度法
    if w_px > 5:
        dist_width = (SIGN_WIDTH_CM * F_PX) / w_px
    else:
        dist_width = -1

    # 面积法
    REF_AREA_PX = 93 * 186
    REF_DIST_CM = 30.0
    if area_px > 100:
        dist_area = REF_DIST_CM * sqrt(REF_AREA_PX / area_px)
    else:
        dist_area = -1

    # 综合决策
    if dist_height > 0:
        distance = dist_height
    elif dist_width > 0:
        distance = dist_width
    elif dist_area > 0:
        distance = dist_area
    else:
        distance = -1

    # 计算横向偏移角度
    center_x = IMAGE_WIDTH / 2
    offset_px = blob.cx() - center_x
    angle_deg = (offset_px / IMAGE_WIDTH) * FOV_H

    if abs(angle_deg) < 3:
        direction = "CENTER"
    elif angle_deg < 0:
        direction = "LEFT"
    else:
        direction = "RIGHT"

    return distance, angle_deg, direction

def draw_blob_info(img, blob, color, label, idx):
    """在图像上绘制交通标志信息"""
    img.draw_rectangle(blob.rect(), color=color, thickness=2)
    dist, angle, direction = estimate_distance_and_angle(blob)
    info_text = "{} #{} D:{:.1f}cm".format(label, idx, dist)
    info_text2 = "Ang:{:.1f}° {}".format(angle, direction)
    img.draw_string(blob.x(), blob.y() - 25, info_text, color=color)
    img.draw_string(blob.x(), blob.y() - 15, info_text2, color=color)
    img.draw_cross(blob.cx(), blob.cy(), color=color, size=5, thickness=2)
    return dist, angle, direction

def draw_line_detection(img, blob, color, label, area):
    """在图像上绘制线条检测结果，显示面积值"""
    img.draw_rectangle(blob.rect(), color=color, thickness=2)
    img.draw_string(blob.x(), blob.y() - 10, "{} Area:{}".format(label, area), color=color)

# ========== 模式处理函数 ==========

def process_clockwise_mode(img):
    """
    顺时针竞速模式：识别橙色线条
    返回：('O', area) 或 ('N', 0)
    """
    img_width = img.width()
    img_height = img.height()
    roi_y = int(img_height * ROI_RATIO)

    orange_blobs = img.find_blobs(
        [ORANGE_THRESHOLD],
        roi=(0, roi_y, img_width, img_height - roi_y),
        merge=False,
        area_threshold=LINE_MIN_AREA
    )

    max_area = 0
    best_blob = None
    for blob in orange_blobs:
        if blob.area() > max_area:
            max_area = blob.area()
            best_blob = blob

    if best_blob:
        draw_line_detection(img, best_blob, (255, 165, 0), "ORANGE", max_area)
        img.draw_string(10, 10, "MODE: CW (ORANGE) Area:{}".format(max_area), color=(255, 165, 0))
        return 'O', max_area
    else:
        img.draw_string(10, 10, "MODE: CW (NO LINE)", color=(255, 255, 255))
        return 'N', 0

def process_anticlockwise_mode(img):
    """
    逆时针竞速模式：识别蓝色线条
    返回：('B', area) 或 ('N', 0)
    """
    img_width = img.width()
    img_height = img.height()
    roi_y = int(img_height * ROI_RATIO)

    blue_blobs = img.find_blobs(
        [BLUE_THRESHOLD],
        roi=(0, roi_y, img_width, img_height - roi_y),
        merge=False,
        area_threshold=LINE_MIN_AREA
    )

    max_area = 0
    best_blob = None
    for blob in blue_blobs:
        if blob.area() > max_area:
            max_area = blob.area()
            best_blob = blob

    if best_blob:
        draw_line_detection(img, best_blob, (0, 0, 255), "BLUE", max_area)
        img.draw_string(10, 10, "MODE: CCW (BLUE) Area:{}".format(max_area), color=(0, 0, 255))
        return 'B', max_area
    else:
        img.draw_string(10, 10, "MODE: CCW (NO LINE)", color=(255, 255, 255))
        return 'N', 0

def process_obstacle_mode(img):
    """
    障碍赛模式：识别红色和绿色交通标志
    返回：('R', valid_red) 或 ('G', valid_green) 或 ('N', [])
    """
    img_width = img.width()
    img_height = img.height()
    roi_y = int(img_height * ROI_RATIO)

    red_blobs = img.find_blobs(
        [RED_THRESHOLD],
        roi=(0, roi_y, img_width, img_height - roi_y),
        merge=False,
        area_threshold=MIN_BLOB_AREA
    )
    green_blobs = img.find_blobs(
        [GREEN_THRESHOLD],
        roi=(0, roi_y, img_width, img_height - roi_y),
        merge=False,
        area_threshold=MIN_BLOB_AREA
    )

    red_blobs = filter_blobs_by_roi(red_blobs, img_width, img_height, ROI_RATIO)
    green_blobs = filter_blobs_by_roi(green_blobs, img_width, img_height, ROI_RATIO)

    valid_red = [b for b in red_blobs if is_valid_blob(b)]
    valid_green = [b for b in green_blobs if is_valid_blob(b)]

    if frame_count % 30 == 0:
        print("Obstacle: R={}, G={}".format(len(valid_red), len(valid_green)))

    # 绘制检测结果（可视化）
    for i, blob in enumerate(valid_red):
        draw_blob_info(img, blob, (255, 0, 0), "RED", i + 1)
    for i, blob in enumerate(valid_green):
        draw_blob_info(img, blob, (0, 255, 0), "GREEN", i + 1)

    img.draw_string(10, 10, "MODE: OBSTACLE", color=(255, 255, 255))

    # 红色优先
    if valid_red:
        return 'R', valid_red
    elif valid_green:
        return 'G', valid_green
    else:
        return 'N', []

# ========== UART 命令接收 ==========

def check_mode_command():
    """检查是否收到 ESP32 发来的模式切换指令"""
    global current_mode, last_mode, mode_updated

    if uart.any():
        data = uart.readline()
        if data:
            try:
                cmd = data.decode().strip().upper()
                if len(cmd) == 1:
                    if cmd in ['C', 'A', 'O', 'I']:
                        if cmd != current_mode:
                            last_mode = current_mode
                            current_mode = cmd
                            mode_updated = True
                            print("Mode changed: {} -> {}".format(last_mode, current_mode))

                            for _ in range(2):
                                led_blue.on()
                                time.sleep_ms(50)
                                led_blue.off()
                                time.sleep_ms(50)

                            mode_names = {
                                'C': 'Clockwise (Orange)',
                                'A': 'Anticlockwise (Blue)',
                                'O': 'Obstacle (Red/Green)',
                                'I': 'Idle'
                            }
                            print("Current mode: {}".format(mode_names.get(current_mode, 'Unknown')))
                    else:
                        print("Invalid command: {}".format(cmd))
            except Exception as e:
                print("UART decode error:", e)

# ========== 主循环 ==========

print("OpenMV Starting...")
print("Waiting for mode command from ESP32...")
print("Commands: C=CW(Orange), A=CCW(Blue), O=Obstacle, I=Idle")

while True:
    img = sensor.snapshot()
    frame_count += 1

    check_mode_command()

    result = 'N'
    area_val = 0

    if current_mode == 'C':
        # 顺时针竞速：识别橙色线条
        result, area_val = process_clockwise_mode(img)
        led_blue.on()
        led_green.on()
        led_red.off()

        if result == 'O':
            send_result('O', area_val)
        else:
            send_result('N', 0)

    elif current_mode == 'A':
        # 逆时针竞速：识别蓝色线条
        result, area_val = process_anticlockwise_mode(img)
        led_green.on()
        led_red.off()

        if result == 'B':
            send_result('B', area_val)
        else:
            send_result('N', 0)

    elif current_mode == 'O':
        # ===== 障碍赛：只发送最近的一个标志 =====
        color, blobs = process_obstacle_mode(img)

        uart.write("TEST,123,45.6\r\n")
        print("Sent test message")

        if color == 'R':
            send_obstacle_data(blobs, 'R')
            led_red.on()
            led_green.off()
        elif color == 'G':
            send_obstacle_data(blobs, 'G')
            led_red.off()
            led_green.on()
        else:
            uart.write('N\r\n')
            print("Sent: N")
            led_red.off()
            led_green.off()

    else:  # 'I' 或未知模式
        img.draw_string(10, 10, "MODE: IDLE", color=(128, 128, 128))
        if frame_count % 60 == 0:
            print("Idle mode, waiting for command...")
        result = 'I'
        led_red.off()
        led_green.off()
        send_result('I')

    time.sleep_ms(50)
