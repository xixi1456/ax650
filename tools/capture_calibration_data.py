#!/usr/bin/env python3
"""
手眼标定数据采集工具
====================
打开海康相机，实时显示画面：
  按 's'  → 保存当前帧为 frame_XXXX.png，同时记录 IMU/云台欧拉角为 euler_XXXX.txt
  按 'q'  → 退出
  按 'p'  → 切换 IMU 数据源（自动探测 UART / 手动输入 / 关闭）

IMU 数据源优先级：
  1. 如果能 import RMCVSerial 或 pyserial + 找到 /dev/ttyUSB*，自动读
  2. 否则按 's' 时手动输入 yaw pitch roll（度，空格分隔）

用法：
  # 方式 1：直接跑（自动探测）
  python3 capture_calibration_data.py

  # 方式 2：指定 UART 端口和波特率
  python3 capture_calibration_data.py --port /dev/ttyUSB0 --baud 921600

  # 方式 3：只开相机，手动输入 IMU
  python3 capture_calibration_data.py --imu manual

  # 方式 4：完全离线（只用手动 IMU）
  python3 capture_calibration_data.py --camera none --imu manual
"""

import os
import sys
import time
import glob
import argparse
import datetime
import threading
import http.server
import socketserver

import cv2
import numpy as np

# ============================================================
# MJPEG HTTP 服务器（远程查看画面）
# ============================================================
LATEST_FRAME = None
FRAME_LOCK = threading.Lock()

class MJPEGHandler(http.server.BaseHTTPRequestHandler):
    """极简 MJPEG / 快照 server, 从主循环共享 LATEST_FRAME"""
    SCALE = 1.0

    def log_message(self, fmt, *args):
        print(f"[HTTP] {args[0]}")

    INDEX_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>Handeye Calibration — Live Stream</title>
<style>
  body {{ margin: 0; background: #1a1a2e; color: #e0e0e0; font-family: -apple-system, "Segoe UI", sans-serif; }}
  .wrap {{ max-width: 1200px; margin: 0 auto; padding: 20px; }}
  h1 {{ font-size: 20px; margin: 0 0 10px 0; color: #64b5f6; }}
  .bar {{ display: flex; gap: 12px; flex-wrap: wrap; margin-bottom: 16px; font-size: 13px; }}
  .chip {{ background: #2d2d44; padding: 6px 12px; border-radius: 20px; }}
  .chip.ok  {{ color: #81c784; }}
  .chip.err {{ color: #ef5350; }}
  a {{ color: #90caf9; text-decoration: none; }}
  a:hover {{ text-decoration: underline; }}
  .stream-box {{ background: #000; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 24px rgba(0,0,0,.6); }}
  img {{ display: block; width: 100%; height: auto; }}
  .links {{ margin-top: 16px; font-size: 13px; line-height: 1.8; }}
  code {{ background: #2d2d44; padding: 2px 6px; border-radius: 4px; }}
</style>
</head>
<body>
<div class="wrap">
  <h1>🎥 Handeye Calibration — Live Preview</h1>
  <div class="bar">
    <span class="chip {cls}">● {state}</span>
    <span class="chip">分辨率: 1440×1080 BayerBG8</span>
    <span class="chip">MJPEG 质量: 80%</span>
  </div>
  <div class="stream-box">
    <img src="/stream" alt="camera stream">
  </div>
  <div class="links">
    🔗 直接链接: <a href="/stream" target="_blank"><code>/stream</code></a> ·
    <a href="/snapshot" target="_blank"><code>/snapshot</code> (单帧)</a> ·
    <a href="/health"><code>/health</code></a><br>
    💡 SSH 转发: <code>ssh -L 8080:localhost:8080 root@&lt;JETSON_IP&gt;</code> 然后在本机打开 <code>http://localhost:8080/</code>
  </div>
</div>
<script>
  setInterval(async () => {{
    try {{
      const r = await fetch('/health', {{ cache: 'no-store' }});
      const t = await r.text();
      document.title = t.includes('True') ? '🎥 Live' : '⏸️ No Frame';
    }} catch(e) {{}}
  }}, 2000);
</script>
</body>
</html>"""

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/' or path == '/index.html':
            self._handle_index()
        elif path == '/stream':
            self._handle_stream()
        elif path == '/snapshot':
            self._handle_snapshot()
        elif path == '/health':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            alive = LATEST_FRAME is not None
            self.wfile.write(f"alive={alive}\n".encode())
        else:
            self.send_response(404)
            self.end_headers()

    def _handle_index(self):
        alive = LATEST_FRAME is not None
        cls = 'ok' if alive else 'err'
        state = 'LIVE (相机正在推流)' if alive else 'NO FRAME (相机未启动或无帧)'
        html = self.INDEX_HTML.format(cls=cls, state=state)
        body = html.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=--frame')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        print(f"[HTTP] /stream 客户端 {self.client_address}")
        try:
            while True:
                with FRAME_LOCK:
                    frame = LATEST_FRAME.copy() if LATEST_FRAME is not None else None
                if frame is None:
                    time.sleep(0.05)
                    continue
                if self.SCALE != 1.0:
                    frame = cv2.resize(frame, None, fx=self.SCALE, fy=self.SCALE)
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
        except BrokenPipeError:
            print(f"[HTTP] /stream 断开 {self.client_address}")
        except Exception as e:
            print(f"[HTTP] /stream 异常: {e}")

    def _handle_snapshot(self):
        with FRAME_LOCK:
            frame = LATEST_FRAME.copy() if LATEST_FRAME is not None else None
        if frame is None:
            self.send_response(503)
            self.end_headers()
            return
        if self.SCALE != 1.0:
            frame = cv2.resize(frame, None, fx=self.SCALE, fy=self.SCALE)
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Disposition', 'inline; filename="snapshot.jpg"')
        self.end_headers()
        self.wfile.write(jpeg.tobytes())


def start_mjpeg_server(port, scale=1.0):
    """后台启动 MJPEG HTTP server"""
    MJPEGHandler.SCALE = scale
    socketserver.TCPServer.allow_reuse_address = True
    try:
        httpd = socketserver.TCPServer(('0.0.0.0', port), MJPEGHandler)
    except OSError as e:
        print(f"[MJPEG] 端口 {port} 被占用，跳过: {e}")
        return None
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return port

# ============================================================
# RMCVSerial 协议 CRC 表 (从 libRMCVSerial.so 反汇编提取)
# CRC8 init=0xFF, CRC16 init=0xFFFF, 与 RMCVSerial 库完全对齐
# ============================================================
_CRC8_TABLE = [
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35,
]
_CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]

def _crc8(data, length=None):
    if length is None: length = len(data)
    crc = 0xFF
    for i in range(length):
        crc = _CRC8_TABLE[(crc ^ data[i]) & 0xFF]
    return crc

def _crc16(data, length=None):
    if length is None: length = len(data)
    crc = 0xFFFF
    for i in range(length):
        # libRMCVSerial.so generate_crc16_checksum 反汇编确认:
        #   idx = (crc ^ byte) & 0xFF
        #   crc = (crc >> 8) ^ table[idx]
        crc = ((crc >> 8) & 0xFFFF) ^ _CRC16_TABLE[(crc ^ data[i]) & 0xFF]
    return crc

# RMCVSerial 协议帧格式:
#   [0] SOF=0xA5  [1-2] Length(小端)  [3] Seq  [4] CRC8(覆盖0-3)
#   [5-6] CmdID(小端)  [7..7+Length-1] Payload  [7+Length .. 8+Length] CRC16(覆盖0..7+Length-1)
# 帧总长 = Length + 9
_CMD_MCU_DATA = 0x1021  # MCU上报姿态

def parse_rmcv_frame(raw_bytes):
    """
    从串口字节流里扫描 RMCVSerial 帧, 成功返回 (cmd_id, payload_bytes) 否则 None
    """
    i = 0
    n = len(raw_bytes)
    while i < n:
        # 找 SOF 0xA5
        if raw_bytes[i] != 0xA5:
            i += 1
            continue
        # 至少需要 9 字节帧头 + payload + CRC16
        if i + 9 > n:
            return None
        length = raw_bytes[i+1] | (raw_bytes[i+2] << 8)
        total_len = length + 9
        if i + total_len > n:
            return None  # 帧不完整
        # CRC8 校验 (覆盖 SOF+Len+Seq = frame[0:4], 结果在 frame[4])
        if _crc8(raw_bytes[i:i+4]) != raw_bytes[i+4]:
            i += 1
            continue
        # CRC16 校验 (覆盖 frame[0:total-2], 结果在 frame[total-2:total])
        if _crc16(raw_bytes[i:i+total_len-2]) != (raw_bytes[i+total_len-2] | (raw_bytes[i+total_len-1] << 8)):
            i += 1
            continue
        # 帧合法
        cmd_id = raw_bytes[i+5] | (raw_bytes[i+6] << 8)
        payload = raw_bytes[i+7:i+7+length]
        return cmd_id, payload
    return None


def find_mvs_sdk():
    """自动定位 MVS Python SDK"""
    candidates = [
        '/opt/MVS/Samples/aarch64/Python/MvImport',
        '/opt/MVS/Samples/x86_64/Python/MvImport',
        '/opt/MVS/Samples/aarch64/Python',
        '/opt/MVS/Samples/Python/MvImport',
    ]
    for p in candidates:
        if os.path.isdir(p):
            return p
    return None

def open_hikvision_camera(dev_num=0):
    """打开海康相机，返回 (cap, tl) 或 (None, None)"""
    mvs_path = find_mvs_sdk()
    if not mvs_path:
        print("[相机] 找不到 MVS Python SDK，回退到 OpenCV")
        return None, None

    sys.path.insert(0, mvs_path)
    os.environ.setdefault('MVCAM_COMMON_RUNENV', '/opt/MVS/lib')
    os.environ.setdefault('LD_LIBRARY_PATH',
                         '/opt/MVS/lib/aarch64:' + os.environ.get('LD_LIBRARY_PATH', ''))

    from ctypes import cast, POINTER
    try:
        from MvCameraControl_class import (
            MvCamera, MV_CC_DEVICE_INFO_LIST, MV_CC_DEVICE_INFO, MV_USB_DEVICE,
        )
        import MvCameraControl_class as _mv
    except ImportError as e:
        print(f"[相机] MvCameraControl_class 导入失败: {e}")
        return None, None

    # 可选设备类型 (有些 SDK 版本没导出, 用 hasattr + 数值兜底)
    # MV_CC_DEVICE_TYPE 枚举: GIGE=1, USB=4, CAML=8, ALL=0xFFFFFFFF
    MV_GIGE_DEVICE = getattr(_mv, 'MV_GIGE_DEVICE', 1)
    MV_CAML_DEV   = getattr(_mv, 'MV_CAML_DEV',   8)
    MV_ALL_DEVICE = getattr(_mv, 'MV_ALL_DEVICE', 0xFFFFFFFF)

    tl = MvCamera()
    stDeviceList = MV_CC_DEVICE_INFO_LIST()
    # 枚举顺序: Gige → USB → CameraLink → 全部兜底
    tried = []
    for dev_type_name, dev_type in [('Gige', MV_GIGE_DEVICE), ('USB', MV_USB_DEVICE),
                                    ('CameraLink', MV_CAML_DEV), ('全部', MV_ALL_DEVICE)]:
        if dev_type in tried:
            continue
        tried.append(dev_type)
        stDeviceList = MV_CC_DEVICE_INFO_LIST()
        ret = tl.MV_CC_EnumDevices(dev_type, stDeviceList)
        if ret == 0 and stDeviceList.nDeviceNum > 0:
            print(f"[相机] 发现 {stDeviceList.nDeviceNum} 台 {dev_type_name} 相机，打开第 {dev_num} 台")
            # 官方 Demo 方式: cast + .contents 把 LP__MV_CC_DEVICE_INFO_ 转成 MV_CC_DEVICE_INFO
            stDevInfo = cast(stDeviceList.pDeviceInfo[dev_num], POINTER(MV_CC_DEVICE_INFO)).contents
            ret = tl.MV_CC_CreateHandle(stDevInfo)
            if ret == 0:
                ret = tl.MV_CC_OpenDevice()
                if ret == 0:
                    tl.MV_CC_SetEnumValue("TriggerMode", 0)
                    tl.MV_CC_StartGrabbing()
                    return tl, 'mvs'
                else:
                    print(f"[相机] OpenDevice 失败 nRet=0x{ret & 0xFFFFFFFF:08X}")
            else:
                print(f"[相机] CreateHandle 失败 nRet=0x{ret & 0xFFFFFFFF:08X}")

    print("[相机] 没有可用的 MVS 相机（Gige/USB/CameraLink/全部 都试过了）")
    return None, None


def read_mvs_frame(tl):
    """从 MVS 相机读一帧，返回 cv2.Mat 或 None"""
    try:
        from MvCameraControl_class import MV_FRAME_OUT
        from ctypes import memset, byref, sizeof
        import numpy as np
        import cv2
    except ImportError:
        return None

    try:
        # 官方 Demo 方式: 构造 MV_FRAME_OUT + memset 清零
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))

        ret = tl.MV_CC_GetImageBuffer(stOutFrame, 1000)
        if ret != 0:
            print(f"[MVS] GetImageBuffer 失败: 0x{ret & 0xFFFFFFFF:08X}")
            return None

        info = stOutFrame.stFrameInfo
        buf = stOutFrame.pBufAddr
        if buf is None:
            print(f"[MVS] pBufAddr is None")
            return None

        width = info.nWidth
        height = info.nHeight
        px = int(info.enPixelType)
        nframe = info.nFrameLen

        # buf 是 ctypes 指针 (8B), 必须用 string_at 解引用
        from ctypes import string_at
        raw = np.frombuffer(string_at(buf, nframe), dtype=np.uint8)
        print(f"[MVS] w={width} h={height} px=0x{px:08X} nFrameLen={nframe} raw_len={len(raw)}")

        # 像素格式转 OpenCV
        if px == 0x01080008:   # BayerRG8
            img = raw.reshape(height, width)
            frame = cv2.cvtColor(img, cv2.COLOR_BayerRG2BGR)
        elif px == 0x01080009: # BayerBG8
            img = raw.reshape(height, width)
            frame = cv2.cvtColor(img, cv2.COLOR_BayerBG2BGR)
        elif px == 0x0108000A: # BayerGB8
            img = raw.reshape(height, width)
            frame = cv2.cvtColor(img, cv2.COLOR_BayerGB2BGR)
        elif px == 0x0108000B: # BayerGR8
            img = raw.reshape(height, width)
            frame = cv2.cvtColor(img, cv2.COLOR_BayerGR2BGR)
        elif px == 0x01080001: # Mono8
            gray = raw.reshape(height, width)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif px == 0x02100007: # RGB8
            frame = raw.reshape(height, width, 3)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif px == 0x02100006: # BGR8
            frame = raw.reshape(height, width, 3).copy()
        else:
            # 兜底: 先 try BayerRG
            img = raw.reshape(height, width)
            frame = cv2.cvtColor(img, cv2.COLOR_BayerRG2BGR)

        tl.MV_CC_FreeImageBuffer(stOutFrame)
        return frame
    except Exception as e:
        return None


def open_opencv_camera(dev_num=0):
    """回退方案：用 OpenCV VideoCapture 打开"""
    cap = cv2.VideoCapture(dev_num)
    if cap.isOpened():
        print(f"[相机] OpenCV VideoCapture 打开 /dev/video{dev_num} 成功")
        return cap, 'opencv'
    return None, None


def read_opencv_frame(cap):
    ok, frame = cap.read()
    if not ok:
        return None
    return frame


# ============================================================
# IMU / 云台欧拉角读取
# ============================================================
class IMUReader:
    """
    三种模式：
      uart:   从 UART 串口读（pyserial 实现）
      manual: 手动输入 yaw pitch roll（按 s 时提示）
      none:   不记录欧拉角
    """
    def __init__(self, mode='auto', port=None, baud=921600):
        self.mode = mode
        self.ser = None
        self.latest = (0.0, 0.0, 0.0)  # yaw, pitch, roll（度）
        self.last_update = 0.0
        self._rx_buf = bytearray()     # 串口接收缓冲（处理粘包）

        if mode in ('auto', 'uart'):
            # 默认 /dev/ttyS4 (Jetson UART1), 不是 ttyUSB*
            if port is None:
                port = '/dev/ttyS4'
            self._try_open_uart(port, baud)
            if self.ser is not None:
                self.mode = 'uart'
            elif mode == 'auto':
                print("[IMU] UART 不可用，自动切换为手动输入模式")
                self.mode = 'manual'
            else:
                print("[IMU] UART 打不开，切为 none 模式")
                self.mode = 'none'
        elif mode == 'manual':
            self.mode = 'manual'
        else:
            self.mode = 'none'

    def _try_open_uart(self, port, baud):
        try:
            import serial as pyserial
        except ImportError:
            print("[IMU] pyserial 未安装，跳过 UART")
            return

        try:
            # 与 RMCVSerial::open(921600, parity=none, 8N1) 对齐
            self.ser = pyserial.Serial(
                port, baud,
                bytesize=pyserial.EIGHTBITS,
                parity=pyserial.PARITY_NONE,
                stopbits=pyserial.STOPBITS_ONE,
                timeout=0.05,
            )
            print(f"[IMU] UART 打开 {port} @ {baud} 8N1 ✅")
        except Exception as e:
            print(f"[IMU] UART 打开 {port} 失败: {e}")
            self.ser = None

    def read(self):
        """返回 (yaw, pitch, roll) 单位：度"""
        if self.mode == 'uart' and self.ser is not None:
            return self._read_uart()
        elif self.mode == 'manual':
            return self._read_manual()
        else:
            return self.latest  # none 模式返回上次值

    def _read_uart(self):
        """
        正式解析 RMCVSerial 协议:
          CMD_MCU_DATA (0x1021) payload = pc_mcu_data_t = {
              float curr_yaw, curr_pitch, curr_roll, shoot_speed;
              uint8_t autoaim_mode;
          }
        """
        try:
            chunk = self.ser.read(512)
            if chunk:
                self._rx_buf.extend(chunk)

            # 循环扫描完整帧
            while True:
                result = parse_rmcv_frame(bytes(self._rx_buf))
                if result is None:
                    break  # 要么没 SOF, 要么帧不完整, 要么 CRC 不对
                cmd_id, payload = result
                # 消费掉 SOF 前的垃圾字节 + 完整帧
                sof_idx = bytes(self._rx_buf).find(b'\xA5')
                # 找到第一帧后, 移到下一帧位置继续扫描
                length = payload.__len__()
                frame_total = length + 9
                del self._rx_buf[:sof_idx + frame_total]

                if cmd_id == _CMD_MCU_DATA and len(payload) >= 17:
                    import struct
                    yaw, pitch, roll, speed, mode = struct.unpack_from('<ffffB', payload, 0)
                    # 合理范围过滤 (防脏数据)
                    if -360 < yaw < 360 and -90 < pitch < 90 and -180 < roll < 180:
                        self.latest = (yaw, pitch, roll)
                        self.last_update = time.time()
                        return self.latest

            # 防止缓冲区无限增长
            if len(self._rx_buf) > 4096:
                self._rx_buf = self._rx_buf[-512:]

            return self.latest
        except Exception:
            return self.latest

    def _read_manual(self):
        try:
            raw = input("  输入 yaw pitch roll（度，空格分隔，直接回车跳过）: ").strip()
            if not raw:
                return self.latest
            parts = raw.split()
            if len(parts) >= 3:
                y, p, r = float(parts[0]), float(parts[1]), float(parts[2])
                self.latest = (y, p, r)
                return self.latest
        except Exception:
            pass
        return self.latest

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass

    def toggle_mode(self):
        """按 'p' 切换模式"""
        cycle = ['uart', 'manual', 'none']
        idx = cycle.index(self.mode) if self.mode in cycle else 0
        self.mode = cycle[(idx + 1) % len(cycle)]
        print(f"[IMU] 切换到模式: {self.mode}")


# ============================================================
# 主循环
# ============================================================
def main():
    global LATEST_FRAME
    parser = argparse.ArgumentParser(description='手眼标定数据采集工具')
    parser.add_argument('--camera', choices=['mvs', 'opencv', 'none'], default='mvs',
                        help='相机类型（默认 mvs）')
    parser.add_argument('--port', default=None, help='UART 端口（默认自动探测）')
    parser.add_argument('--baud', type=int, default=921600, help='UART 波特率')
    parser.add_argument('--imu', choices=['auto', 'uart', 'manual', 'none'], default='auto',
                        help='IMU 数据源（默认 auto）')
    parser.add_argument('--outdir', default='./handeye_calibration_data',
                        help='输出目录')
    parser.add_argument('--stream-port', type=int, default=8080,
                        help='MJPEG 视频流 HTTP 端口（0=不启动）')
    parser.add_argument('--stream-scale', type=float, default=1.0,
                        help='视频流缩放 (0.5=半分辨率省带宽)')
    parser.add_argument('--headless', action='store_true', default=None,
                        help='无 X11 显示时强制 headless（自动检测）')
    parser.add_argument('--auto-save', type=float, default=0,
                        help='headless 模式下每隔 N 秒自动保存一帧（0=手动按回车）')
    args = parser.parse_args()

    # 检测 headless（无 X11 DISPLAY）
    has_display = bool(os.environ.get('DISPLAY'))
    if has_display:
        try:
            cv2.namedWindow('__probe__', cv2.WINDOW_NORMAL)
            cv2.destroyWindow('__probe__')
        except Exception:
            has_display = False

    headless = args.headless if args.headless is not None else not has_display
    if headless:
        print("[HEADLESS] 无 X11 显示，进入 headless 模式（imshow 跳过）")
    args._headless = headless

    # --- 初始化相机 ---
    tl, cam_type = None, None
    frame_reader = None
    if args.camera == 'mvs':
        tl, cam_type = open_hikvision_camera()
        if tl is None:
            tl, cam_type = open_opencv_camera()
    elif args.camera == 'opencv':
        tl, cam_type = open_opencv_camera()
    # else args.camera == 'none'

    if cam_type == 'mvs':
        frame_reader = lambda: read_mvs_frame(tl)
    elif cam_type == 'opencv':
        frame_reader = lambda: read_opencv_frame(tl)
    else:
        frame_reader = None

    # --- 初始化 IMU ---
    imu = IMUReader(mode=args.imu, port=args.port, baud=args.baud)

    # --- 准备输出目录 ---
    os.makedirs(args.outdir, exist_ok=True)
    # 找下一个可用 ID
    existing = glob.glob(os.path.join(args.outdir, 'frame_*.png'))
    if existing:
        next_id = max(int(os.path.basename(p).split('_')[1].split('.')[0]) for p in existing) + 1
    else:
        next_id = 1

    # --- 启动 MJPEG 视频流 (不依赖相机, 有帧就推, 没帧返回 503) ---
    if args.stream_port:
        port = start_mjpeg_server(args.stream_port, args.stream_scale)
        if port:
            # 取本机 IP
            try:
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(('8.8.8.8', 80)); local_ip = s.getsockname()[0]; s.close()
            except Exception:
                local_ip = '<机器人IP>'
            print(f"\n🌐 MJPEG 视频流已启动:")
            print(f"  局域网:   http://{local_ip}:{port}/stream")
            print(f"  快照:     http://{local_ip}:{port}/snapshot")
            print(f"  SSH 转发:  ssh -L {port}:localhost:{port} root@{local_ip}")
            print(f"            然后浏览器打开 http://localhost:{port}/stream\n")

    print(f"\n{'='*60}")
    print(f"输出目录: {os.path.abspath(args.outdir)}")
    print(f"起始编号: frame_{next_id:04d}.png / euler_{next_id:04d}.txt")
    print(f"IMU 模式: {imu.mode}")
    print(f"Headless: {args._headless}")
    if args._headless:
        if args.auto_save > 0:
            print(f"自动保存: 每 {args.auto_save} 秒")
        print("控制: 回车=保存  p=切换IMU  q=退出")
    else:
        print("按键: s=保存  p=切换IMU  q=退出")
    print("=" * 60)

    saved_count = 0
    last_frame = None
    last_save_time = 0.0

    # headless 模式用非阻塞 stdin 读回车
    import select
    import sys as _sys
    def _stdin_has_data():
        return bool(select.select([_sys.stdin], [], [], 0)[0])

    def _do_save(next_id, imu, last_frame, args):
        yaw, pitch, roll = imu.read()
        frame_ok = False
        if last_frame is not None:
            frame_path = os.path.join(args.outdir, f'frame_{next_id:04d}.png')
            cv2.imwrite(frame_path, last_frame)
            frame_ok = True
        euler_path = os.path.join(args.outdir, f'euler_{next_id:04d}.txt')
        with open(euler_path, 'w') as f:
            f.write(f"{yaw:.6f} {pitch:.6f} {roll:.6f}\n")
        status = f"[{next_id}] frame_{next_id:04d}.png + euler_{next_id:04d}.txt → yaw={yaw:.2f} pitch={pitch:.2f} roll={roll:.2f}"
        if not frame_ok:
            status += "  (无帧)"
        print(status)
        return yaw, pitch, roll

    try:
        while True:
            frame = None
            if frame_reader is not None:
                frame = frame_reader()
            elif cam_type == 'none':
                pass

            if frame is not None:
                last_frame = frame.copy()
                with FRAME_LOCK:
                    LATEST_FRAME = frame.copy()

            # 更新 IMU
            if imu.mode == 'uart':
                imu._read_uart()

            if not args._headless:
                # === GUI 模式: imshow + waitKey ===
                if frame is not None:
                    disp = frame.copy()
                    h, w = disp.shape[:2]
                    bar_h = 80
                    info = np.zeros((bar_h, w, 3), dtype=np.uint8)
                    yaw, pitch, roll = imu.latest
                    t_str = datetime.datetime.now().strftime('%H:%M:%S')
                    cv2.putText(info, f"IMU: yaw={yaw:7.2f} pitch={pitch:7.2f} roll={roll:7.2f} ({imu.mode})",
                                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                    cv2.putText(info, f"Saved: {saved_count} | frame_{next_id:04d}.png | {t_str}",
                                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                    disp = np.vstack([disp, info])
                    cv2.imshow('Calibration', disp)

                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('p'):
                    imu.toggle_mode()
                    print(f"[IMU] 切换到 {imu.mode}")
                elif key == ord('s'):
                    _do_save(next_id, imu, last_frame, args)
                    saved_count += 1
                    next_id += 1
            else:
                # === HEADLESS 模式: 回车 / 自动保存 ===
                now = time.time()
                # 自动保存
                if args.auto_save > 0 and (now - last_save_time) >= args.auto_save:
                    _do_save(next_id, imu, last_frame, args)
                    saved_count += 1
                    next_id += 1
                    last_save_time = now
                # 回车手动保存
                if _stdin_has_data():
                    line = _sys.stdin.readline().strip()
                    if line == 'q' or line == 'quit':
                        break
                    elif line == 'p':
                        imu.toggle_mode()
                        print(f"[IMU] 切换到 {imu.mode}")
                    elif line == 's' or line == '':
                        _do_save(next_id, imu, last_frame, args)
                        saved_count += 1
                        next_id += 1
                time.sleep(0.03)

    except KeyboardInterrupt:
        print("\nCtrl+C 收到，退出")
    finally:
        print(f"\n共保存 {saved_count} 对数据到 {os.path.abspath(args.outdir)}")
        if tl is not None and cam_type == 'mvs':
            try:
                tl.MV_CC_StopGrabbing()
                tl.MV_CC_CloseDevice()
                tl.MV_CC_DestroyHandle()
            except Exception:
                pass
        elif cam_type == 'opencv' and tl is not None:
            try:
                tl.release()
            except Exception:
                pass
        try:
            if not args._headless:
                cv2.destroyAllWindows()
        except Exception:
            pass
        imu.close()


if __name__ == '__main__':
    main()
