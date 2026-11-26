#!/usr/bin/env python3
"""
远程查看 auto-aim 相机画面（SSH 端口转发 + 浏览器）
====================================================
海康相机 → MVS SDK 采帧 → HTTP MJPEG 流 → 远程浏览器直接看

用法：
  # 1) 机器人端
  python3 stream_camera.py [--port 8080] [--fps 25] [--scale 1.0]

  # 2) 远程电脑：SSH 端口转发
  ssh -L 8080:localhost:8080 root@<机器人IP>

  # 3) 远程电脑浏览器打开：
  http://localhost:8080/stream
  http://localhost:8080/snapshot
"""

import os
import sys
import time
import argparse
import threading
import http.server
import socketserver
from ctypes import *

import cv2
import numpy as np

# ============================================================
# MVS SDK 初始化（适配 5.x 新版 API）
# ============================================================
MVS = None          # MvCameraControl_class 模块
CAMERA = None       # MvCamera 实例
CAMERA_TYPE = None  # 'mvs' | 'opencv'
FRAME_OBJ = None    # MV_FRAME_OUT（MVS 采帧入参对象）
FRAME = None        # 最新一帧 BGR
FRAME_LOCK = threading.Lock()
FPS_TARGET = 25

def find_mvs_path():
    for p in ['/opt/MVS/Samples/aarch64/Python/MvImport',
              '/opt/MVS/Samples/aarch64/Python',
              '/opt/MVS/Samples/x86_64/Python/MvImport']:
        if os.path.isdir(p):
            return p
    return None

def init_camera():
    global MVS, CAMERA, CAMERA_TYPE, FRAME_OBJ

    mvs_path = find_mvs_path()
    if mvs_path:
        sys.path.insert(0, mvs_path)
        os.environ.setdefault('MVCAM_COMMON_RUNENV', '/opt/MVS/lib')
        os.environ.setdefault('LD_LIBRARY_PATH',
                             '/opt/MVS/lib/aarch64:' + os.environ.get('LD_LIBRARY_PATH', ''))
        try:
            import MvCameraControl_class as _MVS
            MVS = _MVS
            _try_open_mvs()
            if CAMERA is not None:
                return True
        except Exception as e:
            print(f"[相机] MVS 初始化失败: {e}")

    # 回退 OpenCV
    print("[相机] 回退 OpenCV VideoCapture")
    for i in range(4):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            CAMERA = cap
            CAMERA_TYPE = 'opencv'
            print(f"[相机] OpenCV /dev/video{i} ✅")
            return True
    print("[相机] 没有可用相机 ❌")
    return False


def _try_open_mvs():
    global CAMERA, CAMERA_TYPE, FRAME_OBJ
    MVS.MvCamera.MV_CC_Initialize()

    st = MVS.MV_CC_DEVICE_INFO_LIST()
    ret = MVS.MvCamera.MV_CC_EnumDevices(MVS.MV_USB_DEVICE, st)
    if ret != 0 or st.nDeviceNum == 0:
        print(f"[相机] MVS 枚举失败 (ret={ret}, count={st.nDeviceNum})")
        return

    print(f"[相机] MVS 枚举到 {st.nDeviceNum} 台，打开第 0 台")
    cam = MVS.MvCamera()
    # 新版 SDK 需要用 cast 解包设备信息
    dev_info = cast(st.pDeviceInfo[0], POINTER(MVS.MV_CC_DEVICE_INFO)).contents
    ret = cam.MV_CC_CreateHandle(dev_info)
    if ret != 0:
        print(f"[相机] CreateHandle 失败 ret=0x{ret & 0xFFFFFFFF:08X}")
        return

    ret = cam.MV_CC_OpenDevice(MVS.MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print(f"[相机] OpenDevice 失败 ret=0x{ret & 0xFFFFFFFF:08X}")
        return

    ret = cam.MV_CC_SetEnumValue("TriggerMode", MVS.MV_TRIGGER_MODE_OFF)
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print(f"[相机] StartGrabbing 失败 ret=0x{ret & 0xFFFFFFFF:08X}")
        return

    FRAME_OBJ = MVS.MV_FRAME_OUT()
    memset(byref(FRAME_OBJ), 0, sizeof(FRAME_OBJ))
    CAMERA = cam
    CAMERA_TYPE = 'mvs'
    print("[相机] MVS 启动成功 ✅")


def _read_one_mvs():
    """新版 MVS API：MV_CC_GetImageBuffer(frame_obj, timeout) -> ret"""
    ret = CAMERA.MV_CC_GetImageBuffer(FRAME_OBJ, 500)
    if ret != 0 or FRAME_OBJ.pBufAddr is None:
        return None

    info = FRAME_OBJ.stFrameInfo
    w, h, n_len = info.nWidth, info.nHeight, info.nFrameLen
    px = int(info.enPixelType)

    try:
        # string_at 拷贝数据！直接 np.frombuffer(pBufAddr, ...) 会报
        # "buffer is smaller than requested size"，因为 pBufAddr 是 ctypes 指针
        raw_bytes = string_at(FRAME_OBJ.pBufAddr, n_len)
        raw = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(h, w)

        # 常见 Bayer 格式 → OpenCV 处理
        if px == 0x01080009:              # BayerRG8 (实际是 0x01080009!)
            frame = cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
        elif px == 0x0108000B:            # BayerGB8
            frame = cv2.cvtColor(raw, cv2.COLOR_BayerGB2BGR)
        elif px == 0x01080006:            # BayerGR8
            frame = cv2.cvtColor(raw, cv2.COLOR_BayerGR2BGR)
        elif px == 0x01080007:            # BayerBG8
            frame = cv2.cvtColor(raw, cv2.COLOR_BayerBG2BGR)
        elif px == 0x01080001:            # Mono8
            frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        else:
            frame = cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
    except Exception as e:
        print(f"[相机] 像素转换失败 px=0x{px:08X}: {e}")
        frame = None

    CAMERA.MV_CC_FreeImageBuffer(FRAME_OBJ)
    return frame


def _read_one():
    if CAMERA_TYPE == 'mvs':
        return _read_one_mvs()
    elif CAMERA_TYPE == 'opencv':
        ok, f = CAMERA.read()
        return f if ok else None
    return None


def grab_loop():
    global FRAME
    print("[采帧] 线程启动")
    interval = 1.0 / FPS_TARGET
    consec_fail = 0
    try:
        while True:
            t0 = time.time()
            frame = _read_one()
            if frame is not None:
                with FRAME_LOCK:
                    FRAME = frame
                consec_fail = 0
            else:
                consec_fail += 1
                if consec_fail == 10 or consec_fail % 50 == 0:
                    print(f"[采帧] 连续失败 {consec_fail} 次")
            dt = time.time() - t0
            if dt < interval:
                time.sleep(interval - dt)
    except Exception as e:
        print(f"[采帧] 退出: {e}")


# ============================================================
# HTTP MJPEG 服务器
# ============================================================
class MJPEGHandler(http.server.BaseHTTPRequestHandler):
    SCALE = 1.0

    def log_message(self, fmt, *args):
        print(f"[HTTP] {args[0]}")

    def do_GET(self):
        if self.path == '/stream':
            self._handle_stream()
        elif self.path == '/snapshot':
            self._handle_snapshot()
        elif self.path == '/health':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            alive = FRAME is not None
            self.wfile.write(f"camera_alive={alive}\ntype={CAMERA_TYPE}\n".encode())
        else:
            self.send_response(404)
            self.end_headers()

    def _handle_stream(self):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=--frame')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        print(f"[HTTP] /stream 客户端 {self.client_address}")
        try:
            while True:
                with FRAME_LOCK:
                    frame = FRAME.copy() if FRAME is not None else None
                if frame is None:
                    time.sleep(0.05)
                    continue
                if self.SCALE != 1.0:
                    frame = cv2.resize(frame, None, fx=self.SCALE, fy=self.SCALE)
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
        except BrokenPipeError:
            print(f"[HTTP] /stream 断开 {self.client_address}")
        except Exception as e:
            print(f"[HTTP] /stream 异常: {e}")

    def _handle_snapshot(self):
        with FRAME_LOCK:
            frame = FRAME.copy() if FRAME is not None else None
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


def cleanup():
    if CAMERA_TYPE == 'mvs':
        try:
            CAMERA.MV_CC_StopGrabbing()
            CAMERA.MV_CC_CloseDevice()
            CAMERA.MV_CC_DestroyHandle()
        except Exception:
            pass
        try:
            MVS.MvCamera.MV_CC_Finalize()
        except Exception:
            pass
    elif CAMERA_TYPE == 'opencv' and CAMERA is not None:
        CAMERA.release()


def main():
    global FPS_TARGET
    parser = argparse.ArgumentParser(description='远程查看 auto-aim 相机画面')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--fps', type=int, default=25)
    parser.add_argument('--scale', type=float, default=1.0,
                        help='输出缩放 (0.5=半分辨率省带宽)')
    args = parser.parse_args()
    FPS_TARGET = args.fps
    MJPEGHandler.SCALE = args.scale

    if not init_camera():
        sys.exit(1)

    threading.Thread(target=grab_loop, daemon=True).start()

    print("[启动] 等待第一帧...")
    deadline = time.time() + 8
    while time.time() < deadline and FRAME is None:
        time.sleep(0.05)
    if FRAME is None:
        print("[启动] 8 秒内没采到帧")
        cleanup()
        sys.exit(1)
    h, w = FRAME.shape[:2]
    print(f"[启动] 第一帧 OK → {w}×{h} @ {FPS_TARGET}fps")

    socketserver.TCPServer.allow_reuse_address = True
    httpd = socketserver.TCPServer(('0.0.0.0', args.port), MJPEGHandler)
    print(f"\n{'='*50}")
    print(f"相机视频流已启动")
    print(f"  局域网:   http://<机器人IP>:{args.port}/stream")
    print(f"  快照:     http://<机器人IP>:{args.port}/snapshot")
    print(f"  健康检查: http://<机器人IP>:{args.port}/health")
    print(f"  SSH 转发:  ssh -L {args.port}:localhost:{args.port} root@<机器人IP>")
    print(f"            然后浏览器打开 http://localhost:{args.port}/stream")
    print(f"{'='*50}\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nCtrl+C 退出")
        httpd.shutdown()
        cleanup()


if __name__ == '__main__':
    main()
