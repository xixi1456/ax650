#!/usr/bin/env python3
"""
MATLAB Camera Calibration Toolbox → 自瞄 YAML 内参文件

用法 1：交互式（直接贴 MATLAB 输出）
    python3 matlab_calib_to_yml.py

用法 2：命令行传值
    python3 matlab_calib_to_yml.py --fx 1357.8 --fy 1361.3 --cx 647.7 --cy 495.3 \
        --dist -0.1205,-0.0465,-0.0078,-0.0091,0.7448 \
        --out cam_param_my.yml

用法 3：从 .mat 文件读取
    python3 matlab_calib_to_yml.py --mat result.mat

MATLAB 标定结束后会打印：
  cameraMatrix = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
  distCoeffs    = [k1, k2, p1, p2, k3]  (或更长)
把它们贴进来就行。
"""
import argparse, sys, os, re


def fmt_vec(vec):
    """OpenCV FileStorage YAML 的 data 数组格式，逗号分隔"""
    return ", ".join(f"{v:.16e}" for v in vec)


def make_matrix(name, rows, cols, dt, flat_data):
    assert rows * cols == len(flat_data), f"{name}: {rows}x{cols} != {len(flat_data)}"
    data_str = fmt_vec(flat_data)
    return (
        f"{name}: !!opencv-matrix\n"
        f"   rows: {rows}\n"
        f"   cols: {cols}\n"
        f"   dt: {dt}\n"
        f"   data: [ {data_str} ]\n"
    )


def build_yml(K3x3, dist, extra=None):
    """
    K3x3: 3x3 list or numpy array
    dist: list of distortion coeffs (k1,k2,p1,p2,k3 或更多)
    extra: dict，可选 Rcb(3x3) / Tcb(3x1) 手眼外参
    """
    K_flat = [float(v) for row in K3x3 for v in row]
    D_flat = [float(v) for v in dist]

    # 截断多余的畸变系数，最多留 8 项 (OpenCV 支持到 k6+s1..s4)
    if len(D_flat) < 4:
        print("⚠️ 畸变系数少于 4 项，补零到 5 项")
        D_flat += [0.0] * (5 - len(D_flat))

    K_block = make_matrix("K", 3, 3, "d", K_flat)
    D_block = make_matrix("D", 1, len(D_flat), "d", D_flat)

    out = "%YAML:1.0\n---\n"
    out += "# Auto-generated from MATLAB calibration\n"
    out += K_block
    out += D_block

    if extra:
        if "Rcb" in extra:
            R = [float(v) for row in extra["Rcb"] for v in row]
            out += make_matrix("Rcb", 3, 3, "f", R)
        if "Tcb" in extra:
            T = [float(v) for v in extra["Tcb"]]
            out += make_matrix("Tcb", 3, 1, "f", T)

    return out


def parse_matrix_text(text, expect_rows=None, expect_cols=None):
    """
    从 MATLAB 输出文本里解析矩阵，支持：
      [1 2 3; 4 5 6]
      [1,2,3; 4,5,6]
      1 2 3
      4 5 6
    """
    text = text.strip()
    rows = []
    # 去掉外层方括号
    text = re.sub(r"^\[|\]$", "", text.strip())
    # 按分号或换行分行
    for line in re.split(r"[;\n]+", text):
        line = line.strip().strip("[]")
        if not line:
            continue
        # 行内按空白/逗号/制表符分割
        vals = [float(x.strip()) for x in re.split(r"[\s,]+", line) if x.strip()]
        if vals:
            rows.append(vals)
    if expect_rows and len(rows) != expect_rows:
        raise ValueError(f"期望 {expect_rows} 行，实际 {len(rows)} 行: {rows}")
    if expect_cols:
        for i, r in enumerate(rows):
            if len(r) != expect_cols:
                raise ValueError(f"第 {i} 行期望 {expect_cols} 列，实际 {len(r)}: {r}")
    return rows


def interactive():
    print("=" * 56)
    print("  MATLAB 标定 → 自瞄 YAML 内参文件 (Ctrl+C 退出)")
    print("=" * 56)
    print()
    print("把 MATLAB Command Window 里复制出来的 matrix 粘贴进来")
    print("例：  [1.35e3 0 6.47e2; 0 1.36e3 4.95e2; 0 0 1]")
    print()

    print("① 粘贴 cameraMatrix (3x3 内参矩阵 K):")
    try:
        k_txt = input("  K = ")
    except EOFError:
        print("\n退出")
        return
    K = parse_matrix_text(k_txt, expect_rows=3, expect_cols=3)
    fx, fy = K[0][0], K[1][1]
    cx, cy = K[0][2], K[1][2]
    print(f"  → fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")

    print()
    print("② 粘贴 distCoeffs (畸变系数 1xN，空格/逗号分隔都行):")
    try:
        d_txt = input("  dist = ")
    except EOFError:
        return
    D_flat = [float(x) for x in re.split(r"[\s,;\[\]]+", d_txt.strip()) if x]
    print(f"  → {len(D_flat)} 项: {D_flat}")

    out_path = input("\n③ 输出文件名 (默认 cam_param_new.yml): ").strip()
    if not out_path:
        out_path = "cam_param_new.yml"

    yml = build_yml(K, D_flat)
    with open(out_path, "w") as f:
        f.write(yml)
    print(f"\n✅ 已写入 {os.path.abspath(out_path)}")
    print("\n预览:")
    print(yml)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fx", type=float)
    ap.add_argument("--fy", type=float)
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)
    ap.add_argument("--dist", type=str, help="逗号分隔的畸变系数，如 -0.12,-0.04,0,0,0")
    ap.add_argument("--K", type=str, help="完整 3x3 矩阵文本，如 '1 0 cx; 0 fy cy; 0 0 1'")
    ap.add_argument("--mat", type=str, help="从 .mat 文件读取 (需要 scipy)")
    ap.add_argument("--out", "-o", type=str, default="cam_param_new.yml")
    args = ap.parse_args()

    if args.mat:
        from scipy.io import loadmat
        m = loadmat(args.mat)
        keys = {k for k in m.keys() if not k.startswith("_") and k != "None"}
        # 尝试多种常见变量名
        K = None
        for name in ("cameraMatrix", "K", "IntrinsicMatrix", "intrinsics"):
            if name in keys and m[name] is not None and m[name].size > 0:
                K = m[name]; break
        D = None
        for name in ("distCoeffs", "dist", "D", "DistortionCoefficients", "distortion"):
            if name in keys and m[name] is not None and m[name].size > 0:
                D = m[name]; break
        if K is None or D is None:
            print(f".mat 文件里找不到内参/畸变变量，实际变量列表: {sorted(keys)}")
            sys.exit(1)
        K = K.tolist()
        D = D.flatten().tolist()
        print(f"从 {args.mat} 读出: K(3x3)={K[0][0]:.2f}x{K[1][1]:.2f}  dist={D}")
    elif args.K:
        K = parse_matrix_text(args.K, expect_rows=3, expect_cols=3)
        D = [float(x) for x in args.dist.split(",")] if args.dist else []
    elif args.fx is not None:
        K = [
            [args.fx, 0.0, args.cx],
            [0.0, args.fy, args.cy],
            [0.0, 0.0, 1.0],
        ]
        D = [float(x) for x in args.dist.split(",")] if args.dist else []
    else:
        interactive()
        return

    yml = build_yml(K, D)
    with open(args.out, "w") as f:
        f.write(yml)
    print(f"✅ 已写入 {os.path.abspath(args.out)}")
    print(yml)


if __name__ == "__main__":
    main()
