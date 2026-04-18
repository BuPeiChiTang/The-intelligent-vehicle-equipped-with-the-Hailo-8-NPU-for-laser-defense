import time
import serial
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from collections import deque

# ----------------配置区域（按需修改）----------------
SERIAL_PORT = 'COM9'
BAUDRATE = 230400
TIMEOUT = 0.05

# 聚类与跟踪参数（可调）
CLUSTER_EPS = 200.0            # mm：DBSCAN eps（聚类半径）
CLUSTER_MIN_SAMPLES = 2        # DBSCAN min_samples
MATCH_DIST = 300.0             # mm：当前簇质心与历史目标匹配最大距离
MOVEMENT_THRESHOLD = 50.0      # mm：单次位移判定门限（小位移可累积判动）
STOP_CONFIRM_TIME = 2.0        # s：运动物体须停稳 >= 2s 才转为静止
PATH_KEEP_TIME = 3.0           # s：轨迹保留时长（最近 3 秒）
OBJECT_EXPIRE_TIME = 8.0       # s：目标长时间未更新则清除

# 背景与前景判定
BG_WINDOW = 30                 # 圈数用于背景中位数
SMOOTH_WINDOW = 5              # 每角度中位数平滑窗口
DIFF_THRESH_MIN = 120          # 前景差分最小门限（mm）

# 可视化颜色
COLOR_DYNAMIC = 'green'   # 运动点（原始）
COLOR_STATIC = 'blue'     # 静止点（原始）
COLOR_CENTER = 'black'    # 聚类中心
COLOR_PATH = 'red'        # 轨迹颜色

# ----------------初始化（保持你原始解析不变）----------------
preferred_fonts = ['Microsoft YaHei', 'SimHei', 'Songti SC', 'STSong', 'Arial Unicode MS']
from matplotlib.font_manager import fontManager
available_names = {f.name for f in fontManager.ttflist}
chosen = next((name for name in preferred_fonts if name in available_names), 'DejaVu Sans')

matplotlib.rcParams['font.family'] = chosen
matplotlib.rcParams['axes.unicode_minus'] = False
print(f"使用字体: {chosen}")

# 串口
lidar = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
lidar.reset_input_buffer()

# 可视化（极坐标）
plt.ion()
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='polar')
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)
MAX_R = 6000
ax.set_rlim(0, MAX_R)

# scatter：raw, static(blue), moving(green), centers(black), paths(red)
scat_raw = ax.scatter(np.empty((0,2)), np.empty((0,)), s=6, c='0.6', alpha=0.35)
scat_static = ax.scatter([], [], s=8, c=COLOR_STATIC)
scat_moving = ax.scatter([], [], s=8, c=COLOR_DYNAMIC)
scat_centers = ax.scatter([], [], s=36, c=COLOR_CENTER, marker='x')
scat_paths = ax.scatter([], [], s=10, c=COLOR_PATH)

# ---------------- 辅助函数 ----------------
def lidar_single_frame_process(data):
    angle_start = (data[5] * 256 + data[6]) / 100.0
    angle_end = (data[55] * 256 + data[56]) / 100.0
    rotation_time = (data[3] * 256 + data[4])
    speed = 60000000.0 / (rotation_time * 24)
    distances = []
    for i in range(7, 54, 3):
        distances.append(data[i] * 256 + data[i+1])
    return angle_start, angle_end, speed, distances

def read_one_frame_from_serial():
    data = lidar.read(1)
    if len(data) == 0 or data[0] != 0xA5:
        return None
    data = lidar.read(1)
    if len(data) == 0 or data[0] != 0x5A:
        return None
    frame_len_byte = lidar.read(1)
    if len(frame_len_byte) == 0:
        return None
    frame_length = frame_len_byte[0]
    data = lidar.read(frame_length - 3)
    if len(data) != frame_length - 3:
        return None
    full_frame = bytearray([0xA5, 0x5A]) + frame_len_byte + data
    checksum = sum(full_frame[:-1]) & 0xFF
    if checksum != full_frame[-1]:
        return None
    return lidar_single_frame_process(full_frame)

def to_offsets_theta_r(angles_rad, ranges):
    """返回 (N,2) 的 numpy 数组以供 scatter.set_offsets 使用"""
    if len(angles_rad) == 0:
        return np.empty((0,2))
    a = np.array(angles_rad).reshape(-1)
    r = np.array(ranges).reshape(-1)
    return np.column_stack((a, r))

def cartesian_from_polar(angles_rad, ranges):
    a = np.array(angles_rad).reshape(-1)
    r = np.array(ranges).reshape(-1)
    xs = r * np.cos(a)
    ys = r * np.sin(a)
    return xs, ys

def polar_from_cartesian(xs, ys):
    xs = np.asarray(xs).reshape(-1)
    ys = np.asarray(ys).reshape(-1)
    rs = np.hypot(xs, ys)
    thetas = np.arctan2(ys, xs) % (2*np.pi)
    return thetas, rs

# ---------------- 跟踪数据结构 ----------------
# objects: id -> {'centroid':(x,y), 'theta_r':(th,r), 'last_update':t, 'state':'dynamic'/'static', 'stop_timer':s, 'path':deque((x,y,t))}
objects = {}
next_obj_id = 0

# ---------------- 背景与缓存 ----------------
BG_WINDOW = BG_WINDOW
bg_buffers = {}      # angle_degree -> list of recent distances
SMOOTH_WINDOW = SMOOTH_WINDOW
smooth_buffers = {}  # angle_degree -> list of last distances for smoothing

circle_angles = []
circle_ranges = []
accumulated_angle = 0.0
frame_counter = 0
start_time = time.time()

# ---------------- 主循环 ----------------
while True:
    frame = read_one_frame_from_serial()
    if frame is None:
        continue

    angle_start, angle_end, speed, distances = frame

    frame_angle_coverage = angle_end - angle_start
    if frame_angle_coverage < 0:
        frame_angle_coverage += 360.0

    angle_step = frame_angle_coverage / 16.0
    angles_deg = [(angle_start + i * angle_step) % 360 for i in range(16)]
    angles_rad = np.radians(angles_deg)

    # 更新平滑buffers
    for deg, d in zip(angles_deg, distances):
        a_idx = int(round(deg)) % 360
        smooth_buffers.setdefault(a_idx, [])
        smooth_buffers[a_idx].append(d)
        if len(smooth_buffers[a_idx]) > SMOOTH_WINDOW:
            smooth_buffers[a_idx].pop(0)

    # 生成平滑距离
    smooth_distances = []
    for deg in angles_deg:
        a_idx = int(round(deg)) % 360
        vals = smooth_buffers.get(a_idx, [])
        if len(vals) == 0:
            smooth_distances.append(distances[angles_deg.index(deg)])
        else:
            smooth_distances.append(int(np.median(vals)))

    # 更新背景模型 buffers
    frame_counter += 1
    for deg, d in zip(angles_deg, smooth_distances):
        idx = int(round(deg)) % 360
        bg_buffers.setdefault(idx, [])
        bg_buffers[idx].append(d)
        if len(bg_buffers[idx]) > BG_WINDOW:
            bg_buffers[idx].pop(0)

    # 累加到圈缓存
    circle_angles.extend(np.radians(angles_deg))
    circle_ranges.extend(smooth_distances)
    accumulated_angle += frame_angle_coverage

    # 判断完成一圈
    if 360 - 20 < accumulated_angle < 360 + 20:
        ts = time.time() - start_time

        # 1) 背景中位数
        bg_median = {}
        for k, buf in bg_buffers.items():
            if len(buf) >= max(1, int(BG_WINDOW/4)):
                bg_median[k] = int(np.median(buf))

        # 2) 前景掩码（相比背景差异大）
        DIFF_THRESH = max(DIFF_THRESH_MIN, int(MOVEMENT_THRESHOLD*1.5))
        is_foreground = []
        for ang_rad, r in zip(circle_angles, circle_ranges):
            deg = int(round(np.degrees(ang_rad))) % 360
            if deg in bg_median:
                is_fg = abs(r - bg_median[deg]) > DIFF_THRESH
            else:
                is_fg = True
            is_foreground.append(is_fg)

        # 3) 静止 / 运动点分类（静止点 = 非前景；运动候选 = 前景）
        is_foreground_arr = np.array(is_foreground, dtype=bool)
        # convert polar arrays to cartesian for clustering
        xs_all = np.array(circle_ranges) * np.cos(np.array(circle_angles))
        ys_all = np.array(circle_ranges) * np.sin(np.array(circle_angles))
        pts_all = np.column_stack((xs_all, ys_all)) if len(xs_all)>0 else np.empty((0,2))

        moving_pts = pts_all[is_foreground_arr] if pts_all.size>0 else np.empty((0,2))
        static_pts = pts_all[~is_foreground_arr] if pts_all.size>0 else np.empty((0,2))

        # 4) 仅对运动候选点做 DBSCAN 聚类（从而把同一移动物体聚成簇）
        current_centroids = []
        cluster_members_indices = []  # indices relative to moving_pts
        if moving_pts.shape[0] > 0:
            db = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES).fit(moving_pts)
            labels = db.labels_
            for lab in set(labels):
                if lab == -1:
                    continue
                idxs = np.where(labels == lab)[0]
                cluster = moving_pts[idxs]
                cx = float(cluster[:,0].mean())
                cy = float(cluster[:,1].mean())
                current_centroids.append((cx, cy))
                cluster_members_indices.append(idxs.tolist())

        # 5) 跟踪/匹配：把当前质心匹配到历史 objects（nearest centroid）并更新路径/状态
        assigned = {}
        used_obj = set()
        obj_ids = list(objects.keys())
        obj_cent = np.array([objects[o]['centroid'] for o in obj_ids]) if len(obj_ids)>0 else np.empty((0,2))

        for ci, (cx, cy) in enumerate(current_centroids):
            if obj_cent.size == 0:
                assigned[ci] = None
                continue
            dists = np.hypot(obj_cent[:,0] - cx, obj_cent[:,1] - cy)
            min_i = int(np.argmin(dists))
            if dists[min_i] <= MATCH_DIST:
                oid = obj_ids[min_i]
                # ensure one-to-one
                if oid in used_obj:
                    # find next nearest unused within MATCH_DIST
                    order = np.argsort(dists)
                    found = False
                    for k in order:
                        oid2 = obj_ids[k]
                        if oid2 not in used_obj and dists[k] <= MATCH_DIST:
                            assigned[ci] = oid2
                            used_obj.add(oid2)
                            found = True
                            break
                    if not found:
                        assigned[ci] = None
                else:
                    assigned[ci] = oid
                    used_obj.add(oid)
            else:
                assigned[ci] = None

        # create new objects for unmatched clusters
        for ci, (cx, cy) in enumerate(current_centroids):
            oid = assigned.get(ci)
            if oid is None:
                oid = next_obj_id
                next_obj_id += 1
                objects[oid] = {
                    'centroid': (cx, cy),
                    'theta_r': polar_from_cartesian([cx],[cy]),  # store polar rep
                    'last_update': ts,
                    'state': 'dynamic',
                    'stop_timer': 0.0,
                    'path': deque()  # store (x,y,t)
                }
                assigned[ci] = oid

        # update matched objects
        updated_obj_ids = set()
        for ci, (cx, cy) in enumerate(current_centroids):
            oid = assigned.get(ci)
            if oid is None:
                continue
            obj = objects[oid]
            prev_x, prev_y = obj['centroid']
            disp = float(np.hypot(cx - prev_x, cy - prev_y))
            dt = ts - obj['last_update'] if obj['last_update'] is not None else 1e-6
            if dt <= 0:
                dt = 1e-6

            if disp >= MOVEMENT_THRESHOLD:
                obj['state'] = 'dynamic'
                obj['stop_timer'] = 0.0
                obj['path'].append((cx, cy, ts))
            else:
                obj['stop_timer'] += dt
                if obj['stop_timer'] >= STOP_CONFIRM_TIME:
                    obj['state'] = 'static'
                else:
                    if obj['state'] == 'dynamic':
                        obj['path'].append((cx, cy, ts))

            obj['centroid'] = (cx, cy)
            ths, rs = polar_from_cartesian([cx],[cy])
            obj['theta_r'] = (float(ths[0]), float(rs[0]))
            obj['last_update'] = ts

            # trim path
            while obj['path'] and (ts - obj['path'][0][2]) > PATH_KEEP_TIME:
                obj['path'].popleft()

            updated_obj_ids.add(oid)

        # expire objects not updated
        for oid in list(objects.keys()):
            if oid not in updated_obj_ids:
                if ts - objects[oid]['last_update'] >= OBJECT_EXPIRE_TIME:
                    del objects[oid]
                else:
                    # increase stop_timer conservatively
                    objects[oid]['stop_timer'] += (ts - objects[oid]['last_update'])
                    if objects[oid]['stop_timer'] >= STOP_CONFIRM_TIME:
                        objects[oid]['state'] = 'static'

        # 6) 准备可视化数组（确保 shape 正确）
        # raw points (gray)
        raw_offsets = to_offsets_theta_r(circle_angles, circle_ranges)
        scat_raw.set_offsets(raw_offsets)

        # static points (blue) = points not foreground
        if static_pts.size == 0:
            scat_static.set_offsets(np.empty((0,2)))
        else:
            th_s, r_s = polar_from_cartesian(static_pts[:,0], static_pts[:,1])
            scat_static.set_offsets(to_offsets_theta_r(th_s, r_s))

        # moving original points (green) = moving_pts
        if moving_pts.size == 0:
            scat_moving.set_offsets(np.empty((0,2)))
        else:
            th_m, r_m = polar_from_cartesian(moving_pts[:,0], moving_pts[:,1])
            scat_moving.set_offsets(to_offsets_theta_r(th_m, r_m))

        # centers black (show only centers of objects currently in 'dynamic' state)
        center_thetas = []
        center_rs = []
        for oid, obj in objects.items():
            if obj['state'] == 'dynamic':
                th, rr = obj['theta_r']
                center_thetas.append(th)
                center_rs.append(rr)
        if len(center_thetas) == 0:
            scat_centers.set_offsets(np.empty((0,2)))
        else:
            scat_centers.set_offsets(to_offsets_theta_r(center_thetas, center_rs))

        # paths red with decay alpha per point -> build facecolors RGBA
        path_points = []
        path_colors = []
        for oid, obj in objects.items():
            if obj['state'] == 'dynamic':
                for (px, py, pt) in obj['path']:
                    thp, rp = polar_from_cartesian([px], [py])
                    path_points.append((thp[0], rp[0]))
                    # alpha decay: recent = 1, older -> smaller
                    alpha = max(0.05, 1.0 - (ts - pt) / PATH_KEEP_TIME)
                    path_colors.append((1.0, 0.0, 0.0, alpha))  # red with alpha

        if len(path_points) == 0:
            scat_paths.set_offsets(np.empty((0,2)))
            try:
                scat_paths.set_facecolors([])
            except Exception:
                pass
        else:
            arr = np.array(path_points)
            scat_paths.set_offsets(arr)
            # set_facecolors expects RGBA per point
            scat_paths.set_facecolors(np.array(path_colors, dtype=float))

        ax.set_title(f"动态检测（绿=动点，蓝=静点，黑=簇心，红=轨迹）  Speed={speed:.1f} rpm")
        plt.pause(0.01)

        # 清空缓存，准备下一圈
        circle_angles = []
        circle_ranges = []
        accumulated_angle = 0.0

# end while
