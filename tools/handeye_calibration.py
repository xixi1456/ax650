import os
import cv2
import numpy as np
import glob

def eulerAnglesToRotationMatrix(theta_deg):
    yaw, pitch, roll = np.radians(theta_deg)  # Convert degrees to radians

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(pitch), -np.sin(pitch)],
                    [0, np.sin(pitch), np.cos(pitch)]])
    R_y = np.array([[np.cos(yaw), 0, np.sin(yaw)],
                    [0, 1, 0],
                    [-np.sin(yaw), 0, np.cos(yaw)]])
    R_z = np.array([[np.cos(roll), -np.sin(roll), 0],
                    [np.sin(roll), np.cos(roll), 0],
                    [0, 0, 1]])
    R = R_y @ R_x @ R_z 
    return R

def load_gimbal_euler_angles(directory):
    """
    Load gimbal angles and return a dict: {identifier: (yaw, pitch, roll)}
    """
    euler_dict = {}
    for txt_path in glob.glob(os.path.join(directory, 'euler_*.txt')):
        identifier = os.path.splitext(os.path.basename(txt_path))[0].split('_')[1]
        try:
            with open(txt_path, 'r') as f:
                line = f.readline().strip()
                if line:
                    yaw, pitch, roll = map(float, line.split())
                    euler_dict[identifier] = (yaw, pitch, roll)
                    # print(euler_dict[identifier])
        except Exception as e:
            print(f"Error reading {txt_path}: {e}")
    return euler_dict

def find_chessboard_corners(images, board_sizes, square_size, mtx, dist, euler_dict):
    """
    Detect chessboard (try multiple board sizes) and return ONLY matched poses with valid gimbal data.
    """
    rvecs_list = []
    tvecs_list = []
    R_gimbals = []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for img_path in images:
        identifier = os.path.splitext(os.path.basename(img_path))[0].split('_')[1]
        if identifier not in euler_dict:
            print(f"Warning: No pose file for {img_path}")
            continue

        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not read {img_path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners = None
        matched_size = None
        for bs in board_sizes:
            try:
                ret, corners = cv2.findChessboardCornersSB(gray, bs, 0)
            except AttributeError:
                ret, corners = cv2.findChessboardCorners(gray, bs, None)
            if ret:
                matched_size = bs
                break

        if matched_size is None:
            print(f"Chessboard not found in {img_path} (tried {board_sizes})")
            continue

        # 用匹配成功的规格构建 objp
        objp = np.zeros((matched_size[0] * matched_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:matched_size[0], 0:matched_size[1]].T.reshape(-1, 2) * square_size

        # SB 输出已是亚像素精度
        try:
            corners2 = corners
        except:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        _, rvec, tvec = cv2.solvePnP(objp, corners2, mtx, dist)

        rvecs_list.append(rvec)
        tvecs_list.append(tvec)
        R_gimbals.append(eulerAnglesToRotationMatrix(euler_dict[identifier]))
        print(f"✓ {img_path} board={matched_size}  yaw={euler_dict[identifier][0]:.1f}°")

    return R_gimbals, rvecs_list, tvecs_list

if __name__ == "__main__":
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    directory = os.path.normpath(os.path.join(_script_dir, "../handeye_calibration_data/"))
    # 11×10 黑方格 → 内角点规格 (10, 9) 或 (9, 10)，两个方向都试
    board_sizes = [(10, 9), (9, 10)]
    square_size = 0.021  # 方格边长 21 mm（如果不是请改这里）

    # Camera intrinsics — 从 inf1.yml 读取（和自瞄用的一致）
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    cam_yml = os.path.join(_script_dir, "../asset/camParam/cam_param_inf1.yml")
    cam_yml = os.path.normpath(cam_yml)
    fs = cv2.FileStorage(cam_yml, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f"❌ 打不开相机参数: {cam_yml}")
        exit(1)
    mtx = fs.getNode("K").mat()
    dist = fs.getNode("D").mat()
    fs.release()
    print(f"[内参] 从 {cam_yml} 读取")
    print(f"  K = {mtx}")
    print(f"  D = {dist}")

    # Load gimbal poses as dict
    euler_dict = load_gimbal_euler_angles(directory)

    # Get sorted image list
    images = sorted(glob.glob(os.path.join(directory, 'frame_*.png')))

    # Match and extract valid poses
    R_gimbals, rvecs, tvecs = find_chessboard_corners(images, board_sizes, square_size, mtx, dist, euler_dict)

    print(f"Valid pose pairs: {len(R_gimbals)}")

    if len(R_gimbals) < 3:
        print("Error: Need at least 3 valid pose pairs for hand-eye calibration!")
        exit(1)

    # ============================================================
    # RoboMaster 自瞄手眼标定：Eye-in-hand
    #
    # 物理模型：
    #   [相机] ──固定── [IMU]    ← 都固定在云台上（gripper，动端）
    #        │             │
    #      云台转轴
    #   ────────────────────
    #        │
    #      云台基座 / 车体      ← 固定（base = world，不动）
    #
    #   标定板 ← 外部世界里，不动
    #
    # 求 cam → IMU（CoordTransformer 需要 R_c2i / T_c2i）
    # ============================================================
    # R_gripper2base = imu→world = inv(R_world2imu) = R_world2imu.T
    # t_gripper2base = 全零（云台只有旋转，没有平移）
    R_gripper2base = np.array([R.T for R in R_gimbals])  # 对 world→imu 旋转转置
    t_gripper2base = np.zeros((len(R_gimbals), 3))

    # rvecs/tvecs 是 solvePnP 返回的 target→cam（标定板→相机），正好符合 calibrateHandEye 要求

    methods = [
        ("TSAI",       cv2.CALIB_HAND_EYE_TSAI),
        ("PARK",       cv2.CALIB_HAND_EYE_PARK),
        ("HORAUD",     cv2.CALIB_HAND_EYE_HORAUD),
        ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]

    best = None
    for name, m in methods:
        try:
            R_cam2imu, t_cam2imu = cv2.calibrateHandEye(R_gripper2base, t_gripper2base,
                                                        rvecs, tvecs, method=m)
            is_trivial = (np.allclose(R_cam2imu, np.eye(3), atol=0.02) and
                          np.allclose(t_cam2imu, 0, atol=0.002))
            tmm = t_cam2imu.flatten() * 1000
            if is_trivial:
                print(f"  {name:12s}: ❌ 平凡解  t=[{tmm[0]:+.0f},{tmm[1]:+.0f},{tmm[2]:+.0f}]mm")
                continue
            # 评分：t 幅度 + R 偏离恒等程度（越大越好）
            score = np.linalg.norm(t_cam2imu) * 1000 + np.linalg.norm(R_cam2imu - np.eye(3))
            print(f"  {name:12s}: ✅ t=[{tmm[0]:+.0f},{tmm[1]:+.0f},{tmm[2]:+.0f}]mm  评分={score:.4f}")
            if best is None or score > best[0]:
                best = (score, name, R_cam2imu, t_cam2imu)
        except cv2.error as e:
            msg = str(e.err) if e.err else str(e)
            msg = msg.split('\n')[-1]
            print(f"  {name:12s}: ❌ {msg}")

    if best is None:
        print("\n❌ 所有方法都没解出来！需要更多云台转动（至少每帧 20°+ 差异）")
        exit(1)

    R_c2i, T_c2i = best[2], best[3]
    print(f"\n🎯 采用方法: {best[1]}  (评分={best[0]:.4f})")

    print("=" * 60)
    print("C++ CoordTransformer 需要的 cam → IMU")
    print("=" * 60)
    print("R_c2i (直接输出，eye-in-hand 模式下不需要额外逆变换):")
    print(R_c2i)
    tmm = T_c2i.flatten() * 1000
    print(f"T_c2i = [{tmm[0]:+.1f}, {tmm[1]:+.1f}, {tmm[2]:+.1f}] mm")

    # OpenCV YAML matrix 格式
    def _to_yml_matrix(mat, dtype='f'):
        rows, cols = mat.shape
        flat = mat.flatten()
        data_str = ', '.join(f'{v:.8f}' for v in flat)
        return (
            f"!!opencv-matrix\n"
            f"   rows: {rows}\n"
            f"   cols: {cols}\n"
            f"   dt: {dtype}\n"
            f"   data: [ {data_str} ]"
        )

    print()
    print("=" * 60)
    print("OpenCV YAML 格式（可直接复制到 cam_param_*.yml）")
    print("=" * 60)
    print("R_c2i:")
    print(_to_yml_matrix(R_c2i, 'f'))
    print()
    print("T_c2i:")
    print(_to_yml_matrix(T_c2i.reshape(3, 1), 'f'))