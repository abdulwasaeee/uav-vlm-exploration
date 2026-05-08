# %% [markdown]
# # UAV ROS2 Bag Analyser
#
# Open in VS Code → "Run Cell" buttons appear automatically.
# Or: `jupyter nbconvert --to notebook --execute bag_analysis.py`
#
# **Topics parsed:**
# - `/uav/mp_diag`            — planner diagnostics (12 fields)
# - `/uav/vfh_status`         — planner state string
# - `/drone/odom`             — drone pose (ENU)
# - `/uav/cmd_vel`            — commanded velocity
# - `/uav/mission_complete`   — bool
# - `/drone/tof_merged/points`— merged point cloud (if `--with-cloud`)

# %% [markdown]
# ## 1 · Configuration

# %%
import os, glob, struct, math, sqlite3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize
from matplotlib import cm

# ── Bag selection ──────────────────────────────────────────────────────────
# Set to a specific path, or leave None to auto-pick the most recent bag.
BAG_PATH = None
# BAG_PATH = "/home/shantam/irobot/planner_ws/bags/pole_avoidv3_20260508_XXXXXX"

BAGS_DIR = os.path.expanduser("~/irobot/planner_ws/bags")

if BAG_PATH is None:
    bags = sorted(glob.glob(os.path.join(BAGS_DIR, "*")), key=os.path.getmtime)
    if not bags:
        raise FileNotFoundError(f"No bags found in {BAGS_DIR}")
    BAG_PATH = bags[-1]

DB_PATH = glob.glob(os.path.join(BAG_PATH, "*.db3"))
if not DB_PATH:
    raise FileNotFoundError(f"No .db3 file in {BAG_PATH}")
DB_PATH = DB_PATH[0]

print(f"Bag : {BAG_PATH}")
print(f"DB  : {DB_PATH}")

# Known world geometry (indoor_obstacle.sdf)
POLES = [(7.0, 0.0, 0.12), (7.0, 2.0, 0.12), (7.0, 4.0, 0.12)]  # (x, y, radius)
GOAL  = (9.0, 0.0, 1.5)

# Planner params used when bag was recorded
COLLISION_RADIUS = 0.75
MIN_CLEARANCE    = 1.0
ARC_LENGTH       = 2.0

# %% [markdown]
# ## 2 · CDR Reader + Message Parsers

# %%
class CDRReader:
    """
    Little-endian CDR reader for ROS2 serialised messages.
    Alignment is always computed relative to byte 4 (end of encapsulation header).
    """
    def __init__(self, buf):
        self.buf = bytes(buf)
        self.pos = 4  # skip 4-byte CDR encapsulation header

    def _align(self, n):
        rem = (self.pos - 4) % n
        if rem:
            self.pos += n - rem

    def u8(self):
        v = struct.unpack_from('B', self.buf, self.pos)[0]
        self.pos += 1
        return v

    def i32(self):
        self._align(4)
        v = struct.unpack_from('<i', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def u32(self):
        self._align(4)
        v = struct.unpack_from('<I', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def f64(self):
        self._align(8)
        v = struct.unpack_from('<d', self.buf, self.pos)[0]
        self.pos += 8
        return v

    def string(self):
        """ROS2 CDR string: uint32 length (incl. null) + bytes."""
        n = self.u32()
        s = self.buf[self.pos:self.pos + n].rstrip(b'\x00').decode('utf-8', errors='replace')
        self.pos += n
        return s

    def raw(self, n):
        v = self.buf[self.pos:self.pos + n]
        self.pos += n
        return v


# ── Diag field names (layout from mp_node.cpp) ────────────────────────────
DIAG_FIELDS = [
    'dist_goal',   # [0]  distance to current waypoint (m)
    'yaw',         # [1]  drone yaw in map frame (rad)
    'yaw_accum',   # [2]  orbit detector accumulated yaw (rad)
    'orbiting',    # [3]  orbit flag (0/1)
    'stalled',     # [4]  stall flag (0/1)
    'bypass',      # [5]  bypass state: 0=NONE 1=LEFT 2=RIGHT
    'stall_best',  # [6]  stall: best-ever dist to goal (m)
    'stall_since', # [7]  stall: seconds since last improvement
    'obs_d',       # [8]  closest obstacle distance (m)
    'best_prim',   # [9]  index of selected horizontal primitive (-1=none)
    'estop',       # [10] e-stop flag (0/1)
    'obs_det',     # [11] obstacle_detected flag (0/1)
]


def parse_float64array(buf):
    """std_msgs/Float64MultiArray → list of floats."""
    try:
        r = CDRReader(buf)
        r.u32()   # dim_length (0 for flat arrays)
        r.u32()   # data_offset
        n = r.u32()
        return [r.f64() for _ in range(n)]
    except Exception:
        return []


def parse_string_msg(buf):
    """std_msgs/String → str."""
    try:
        return CDRReader(buf).string()
    except Exception:
        return ''


def parse_odometry(buf):
    """
    nav_msgs/Odometry → (t, x, y, z, yaw).
    Position is in the message frame (map/ENU for this stack).
    """
    try:
        r = CDRReader(buf)
        sec  = r.i32()
        nsec = r.u32()
        r.string()   # header.frame_id
        r.string()   # child_frame_id
        x = r.f64(); y = r.f64(); z = r.f64()
        qx = r.f64(); qy = r.f64(); qz = r.f64(); qw = r.f64()
        t = sec + nsec * 1e-9
        if abs(x) < 300 and abs(y) < 300 and abs(z) < 100:
            yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
            return t, x, y, z, yaw
    except Exception:
        pass
    return None


def parse_twist_stamped(buf):
    """geometry_msgs/TwistStamped → (t, vx, vy, vz)."""
    try:
        r = CDRReader(buf)
        sec  = r.i32()
        nsec = r.u32()
        r.string()   # frame_id
        vx = r.f64(); vy = r.f64(); vz = r.f64()
        return sec + nsec * 1e-9, vx, vy, vz
    except Exception:
        return None


def parse_bool_msg(buf):
    """std_msgs/Bool → bool."""
    try:
        return bool(CDRReader(buf).u8())
    except Exception:
        return False


# PointCloud2 datatype codes → (struct format char, byte size)
_PC2_DT = {1: ('b',1), 2: ('B',1), 3: ('h',2), 4: ('H',2),
           5: ('i',4), 6: ('I',4), 7: ('f',4), 8: ('d',8)}


def parse_pointcloud2(buf):
    """
    sensor_msgs/PointCloud2 → (t, frame_id, xs, ys, zs) as numpy arrays.
    Points with non-finite values are dropped.
    """
    try:
        r = CDRReader(buf)
        sec  = r.i32(); nsec = r.u32(); frame_id = r.string()
        height = r.u32(); width = r.u32()
        n_fields = r.u32()
        fields = {}
        for _ in range(n_fields):
            name     = r.string()
            offset   = r.u32()
            datatype = r.u8()
            count    = r.u32()
            fields[name] = (offset, datatype)

        r.u8()            # is_bigendian
        point_step = r.u32()
        r.u32()            # row_step
        data_len   = r.u32()
        raw_data   = r.raw(data_len)
        t = sec + nsec * 1e-9

        n_pts = height * width
        x_off, x_dt = fields.get('x', (0,  7))
        y_off, y_dt = fields.get('y', (4,  7))
        z_off, z_dt = fields.get('z', (8,  7))
        x_fmt = '<' + _PC2_DT[x_dt][0]
        y_fmt = '<' + _PC2_DT[y_dt][0]
        z_fmt = '<' + _PC2_DT[z_dt][0]

        xs, ys, zs = [], [], []
        for i in range(n_pts):
            b = i * point_step
            xv = struct.unpack_from(x_fmt, raw_data, b + x_off)[0]
            yv = struct.unpack_from(y_fmt, raw_data, b + y_off)[0]
            zv = struct.unpack_from(z_fmt, raw_data, b + z_off)[0]
            if math.isfinite(xv) and math.isfinite(yv) and math.isfinite(zv):
                xs.append(xv); ys.append(yv); zs.append(zv)

        return t, frame_id, np.array(xs, dtype=np.float32), \
                             np.array(ys, dtype=np.float32), \
                             np.array(zs, dtype=np.float32)
    except Exception as e:
        return None

# %% [markdown]
# ## 3 · Load Bag

# %%
conn = sqlite3.connect(DB_PATH)
cur  = conn.cursor()

cur.execute("SELECT id, name, type FROM topics")
topic_info = {row[1]: (row[0], row[2]) for row in cur.fetchall()}

print("Topics in bag:")
for name, (tid, ttype) in sorted(topic_info.items()):
    cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,))
    n = cur.fetchone()[0]
    print(f"  {name:50s}  {n:5d} msgs   ({ttype})")

HAS_CLOUD = '/drone/tof_merged/points' in topic_info
print(f"\nCloud data available: {'YES' if HAS_CLOUD else 'NO (re-record with --with-cloud)'}")

# %% [markdown]
# ## 4 · Parse All Topics

# %%
def fetch_topic(name, parser):
    if name not in topic_info:
        return []
    tid = topic_info[name][0]
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
    out = []
    for ts, data in cur.fetchall():
        r = parser(bytes(data))
        if r is not None:
            out.append((ts * 1e-9, r))
    return out


# ── Diag ──────────────────────────────────────────────────────────────────
_diag_raw = fetch_topic('/uav/mp_diag',
    lambda b: parse_float64array(b) if len(parse_float64array(b)) >= 12 else None)

diag_t = np.array([ts for ts, _ in _diag_raw])
diag_v = np.array([v  for _,  v in _diag_raw])  # shape (N, 12)

t0 = diag_t[0] if len(diag_t) else 0.0
diag_t -= t0

D = {name: diag_v[:, i] for i, name in enumerate(DIAG_FIELDS)}
print(f"Diag  : {len(diag_t):4d} samples  duration={diag_t[-1]:.1f}s")

# ── Status ────────────────────────────────────────────────────────────────
_status_raw = fetch_topic('/uav/vfh_status', parse_string_msg)
status_rows = [(ts - t0, s) for ts, s in _status_raw]
status_set  = set(s for _, s in status_rows)
print(f"Status: {len(status_rows):4d} samples  values={status_set}")

# ── Odometry ──────────────────────────────────────────────────────────────
_odom_raw = fetch_topic('/drone/odom', parse_odometry)
odom_rows  = [(ts - t0, r[0], r[1], r[2], r[3], r[4]) for ts, r in _odom_raw]
# columns: t_rel, t_abs, x, y, z, yaw
# (t_abs is the message stamp, t_rel is wall-clock relative to first diag)
odom_t = np.array([r[0] for r in odom_rows])
odom_x = np.array([r[2] for r in odom_rows])
odom_y = np.array([r[3] for r in odom_rows])
odom_z = np.array([r[4] for r in odom_rows])
odom_yaw = np.array([r[5] for r in odom_rows])
print(f"Odom  : {len(odom_rows):4d} valid samples")

# ── Cmd vel ───────────────────────────────────────────────────────────────
_cmd_raw = fetch_topic('/uav/cmd_vel', parse_twist_stamped)
cmd_t     = np.array([ts - t0          for ts, r in _cmd_raw])
cmd_speed = np.array([math.hypot(r[1], math.hypot(r[2], r[3])) for _, r in _cmd_raw])
cmd_vx    = np.array([r[1] for _, r in _cmd_raw])
cmd_vy    = np.array([r[2] for _, r in _cmd_raw])
print(f"CmdVel: {len(cmd_t):4d} samples")

# ── Cloud (optional) ──────────────────────────────────────────────────────
cloud_rows = []
if HAS_CLOUD:
    tid = topic_info['/drone/tof_merged/points'][0]
    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
    for ts, data in cur.fetchall():
        r = parse_pointcloud2(bytes(data))
        if r is not None:
            cloud_rows.append((ts * 1e-9 - t0,) + r[1:])
    npts = [len(r[2]) for r in cloud_rows]
    print(f"Cloud : {len(cloud_rows):4d} frames  pts/frame: "
          f"min={min(npts)} median={int(np.median(npts))} max={max(npts)}")

# ── Mission complete ──────────────────────────────────────────────────────
mission_done = False
if '/uav/mission_complete' in topic_info:
    tid = topic_info['/uav/mission_complete'][0]
    cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 1", (tid,))
    row = cur.fetchone()
    if row:
        mission_done = parse_bool_msg(bytes(row[0]))
print(f"\nMission complete: {mission_done}")

# %% [markdown]
# ## 5 · Timeline Overview

# %%
STATUS_COLORS = {
    'NOMINAL':   '#2ecc71',  # green
    'AVOIDING':  '#f39c12',  # orange
    'AVOIDING_L':'#e67e22',  # dark orange
    'AVOIDING_R':'#e74c3c',  # red-orange
    'ESTOP':     '#c0392b',  # red
    'STALLED':   '#8e44ad',  # purple
    'ORBITING':  '#795548',  # brown
    'IDLE':      '#95a5a6',  # gray
}


def shade_status(ax, t_end):
    """Fill axis background with planner state colours."""
    if not status_rows:
        return
    prev_t, prev_s = status_rows[0]
    for t, s in status_rows[1:]:
        c = STATUS_COLORS.get(prev_s, '#dddddd')
        ax.axvspan(prev_t, t, alpha=0.12, color=c, linewidth=0)
        prev_t, prev_s = t, s
    ax.axvspan(prev_t, t_end, alpha=0.12,
               color=STATUS_COLORS.get(prev_s, '#dddddd'), linewidth=0)


t_end = diag_t[-1]
fig = plt.figure(figsize=(15, 11))
gs  = gridspec.GridSpec(4, 1, hspace=0.45)

# ── Subplot 1: obstacle distance ──────────────────────────────────────────
ax1 = fig.add_subplot(gs[0])
shade_status(ax1, t_end)
ax1.plot(diag_t, D['obs_d'], color='steelblue', lw=1.3, label='obs_d')
ax1.axhline(MIN_CLEARANCE,    color='red',    ls='--', lw=1.2, alpha=0.8,
            label=f'min_clearance ({MIN_CLEARANCE}m)')
ax1.axhline(ARC_LENGTH,       color='orange', ls='--', lw=1.0, alpha=0.7,
            label=f'arc_length ({ARC_LENGTH}m)')
ax1.axhline(COLLISION_RADIUS, color='purple', ls=':',  lw=1.0, alpha=0.7,
            label=f'collision_radius ({COLLISION_RADIUS}m)')
estop_mask = D['estop'] > 0.5
if estop_mask.any():
    ax1.scatter(diag_t[estop_mask],
                np.full(estop_mask.sum(), 0.05),
                c='red', s=18, zorder=5, label='ESTOP tick')
ax1.set_ylabel('Distance (m)')
ax1.set_ylim(bottom=0, top=min(2.6, D['obs_d'].max() * 1.05))
ax1.legend(fontsize=7, loc='upper right', ncol=2)
ax1.set_title('Closest Obstacle Distance', fontsize=9)
ax1.grid(True, alpha=0.3)

# ── Subplot 2: distance to goal ───────────────────────────────────────────
ax2 = fig.add_subplot(gs[1], sharex=ax1)
shade_status(ax2, t_end)
ax2.plot(diag_t, D['dist_goal'], color='navy', lw=1.3)
ax2.axhline(0.25, color='gold', ls='--', lw=1, alpha=0.7, label='accept radius')
ax2.set_ylabel('Distance (m)')
ax2.legend(fontsize=7, loc='upper right')
ax2.set_title('Distance to Goal', fontsize=9)
ax2.grid(True, alpha=0.3)

# ── Subplot 3: bypass state + best primitive ──────────────────────────────
ax3 = fig.add_subplot(gs[2], sharex=ax1)
shade_status(ax3, t_end)
ax3.plot(diag_t, D['bypass'], color='darkgreen', lw=2.0,
         drawstyle='steps-post', label='bypass')
ax3.set_yticks([0, 1, 2])
ax3.set_yticklabels(['NONE', 'LEFT', 'RIGHT'])
ax3.set_ylabel('Bypass', color='darkgreen')
ax3_r = ax3.twinx()
ax3_r.plot(diag_t, D['best_prim'], color='#888888', lw=0.8, alpha=0.6)
ax3_r.set_ylabel('Best prim idx', color='#888888', fontsize=8)
ax3.set_title('Bypass State + Selected Primitive', fontsize=9)
ax3.grid(True, alpha=0.3)

# ── Subplot 4: commanded speed ────────────────────────────────────────────
ax4 = fig.add_subplot(gs[3], sharex=ax1)
shade_status(ax4, t_end)
ax4.plot(cmd_t, cmd_speed, color='teal', lw=1.3)
ax4.set_ylabel('Speed (m/s)')
ax4.set_xlabel('Time relative to recording start (s)')
ax4.set_title('Commanded Velocity Magnitude', fontsize=9)
ax4.grid(True, alpha=0.3)

# Status legend (shared)
legend_patches = [
    mpatches.Patch(facecolor=c, alpha=0.6, label=s)
    for s, c in STATUS_COLORS.items() if s in status_set
]
ax1.legend(handles=ax1.get_legend_handles_labels()[0] + legend_patches,
           fontsize=7, loc='upper right', ncol=3)

fig.suptitle(f"Timeline — {os.path.basename(BAG_PATH)}", fontsize=11)
out_path = os.path.join(BAG_PATH, 'timeline.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
plt.show()
print(f"Saved → {out_path}")

# %% [markdown]
# ## 6 · Drone Trajectory

# %%
if len(odom_rows) < 5:
    print("Insufficient odometry data — skipping trajectory plot.")
else:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"Drone Trajectory — {os.path.basename(BAG_PATH)}", fontsize=11)

    norm = Normalize(vmin=odom_t.min(), vmax=odom_t.max())
    cmap = cm.viridis

    # ── XY top-down ───────────────────────────────────────────────────────
    ax = axes[0]
    sc = ax.scatter(odom_x, odom_y, c=odom_t, cmap=cmap, norm=norm, s=6, zorder=3)
    plt.colorbar(sc, ax=ax, label='Time (s)', shrink=0.85)

    for px, py, pr in POLES:
        ax.add_patch(plt.Circle((px, py), pr,            color='red',  alpha=0.8, zorder=5))
        ax.add_patch(plt.Circle((px, py), COLLISION_RADIUS,
                                color='red', alpha=0.15, fill=True,   zorder=4))
        ax.add_patch(plt.Circle((px, py), COLLISION_RADIUS,
                                color='red', fill=False, lw=1.0, ls='--', zorder=4))

    ax.scatter(*GOAL[:2],   s=250, c='gold',  marker='*', zorder=6, label='Goal')
    ax.scatter(odom_x[0], odom_y[0], s=120, c='lime', marker='o', zorder=6, label='Start')
    ax.scatter(odom_x[-1], odom_y[-1], s=120, c='cyan', marker='s', zorder=6, label='End')
    ax.set_xlabel('X (m, East)'); ax.set_ylabel('Y (m, North)')
    ax.set_title('XY — top-down')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # ── XZ side view ──────────────────────────────────────────────────────
    ax = axes[1]
    sc = ax.scatter(odom_x, odom_z, c=odom_t, cmap=cmap, norm=norm, s=6, zorder=3)
    plt.colorbar(sc, ax=ax, label='Time (s)', shrink=0.85)
    ax.axhline(GOAL[2], color='gold', ls='--', lw=1.2, label=f'Goal z={GOAL[2]}m')
    ax.axhline(MIN_CLEARANCE, color='orange', ls=':', lw=1.0, alpha=0.6,
               label=f'min_clearance ({MIN_CLEARANCE}m)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Z / altitude (m)')
    ax.set_title('XZ — side view')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    plt.tight_layout()
    out_path = os.path.join(BAG_PATH, 'trajectory.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Saved → {out_path}")

# %% [markdown]
# ## 7 · Point Cloud Snapshots
#
# Shows the merged ToF cloud (base_link frame) at three moments.
# Drone is always at the origin.  Adjust `SNAP_TIMES` to any times of interest.

# %%
if not HAS_CLOUD:
    print("No cloud in bag. Re-record with --with-cloud to enable this section.")
elif not cloud_rows:
    print("Cloud topic present but no frames parsed.")
else:
    cloud_ts = np.array([r[0] for r in cloud_rows])

    # ── Pick interesting snap times automatically ─────────────────────────
    estop_times = diag_t[D['estop'] > 0.5]
    if len(estop_times):
        t_e = estop_times[0]
        SNAP_TIMES  = [max(0, t_e - 3.0), t_e, min(t_end, t_e + 1.5)]
        SNAP_LABELS = [f'Pre-ESTOP (t={SNAP_TIMES[0]:.1f}s)',
                       f'At ESTOP  (t={SNAP_TIMES[1]:.1f}s)',
                       f'Post-ESTOP(t={SNAP_TIMES[2]:.1f}s)']
    else:
        n = len(cloud_rows)
        SNAP_TIMES  = [cloud_ts[n//4], cloud_ts[n//2], cloud_ts[3*n//4]]
        SNAP_LABELS = [f't={t:.1f}s' for t in SNAP_TIMES]

    # Override here if you want specific times:
    # SNAP_TIMES  = [5.0, 10.0, 15.0]
    # SNAP_LABELS = ['t=5s', 't=10s', 't=15s']

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle(f"Point Cloud Snapshots (base_link frame) — {os.path.basename(BAG_PATH)}",
                 fontsize=11)

    for ax, t_snap, label in zip(axes, SNAP_TIMES, SNAP_LABELS):
        idx = int(np.argmin(np.abs(cloud_ts - t_snap)))
        _, frame_id, xs, ys, zs = cloud_rows[idx]
        t_actual = cloud_ts[idx]

        if len(xs) == 0:
            ax.text(0.5, 0.5, 'No valid points', transform=ax.transAxes, ha='center')
            ax.set_title(label); continue

        # Top-down XY, coloured by height (z)
        z_lo, z_hi = -1.2, 1.2
        sc = ax.scatter(xs, ys, c=np.clip(zs, z_lo, z_hi), cmap='RdYlBu_r',
                        s=1.5, alpha=0.7, vmin=z_lo, vmax=z_hi)
        plt.colorbar(sc, ax=ax, label='Z (m)', shrink=0.8, pad=0.02)

        # Drone origin
        ax.scatter(0, 0, s=120, c='white', marker='+', zorder=6, linewidths=2)
        # Safety circles
        ax.add_patch(plt.Circle((0,0), COLLISION_RADIUS, color='lime',
                                fill=False, lw=1.5, ls='-',  zorder=5,
                                label=f'collision_r ({COLLISION_RADIUS}m)'))
        ax.add_patch(plt.Circle((0,0), MIN_CLEARANCE,    color='red',
                                fill=False, lw=1.5, ls='--', zorder=5,
                                label=f'min_clear ({MIN_CLEARANCE}m)'))

        lim = max(3.0, COLLISION_RADIUS * 2.5)
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m, fwd)'); ax.set_ylabel('Y (m, left)')
        ax.set_title(f'{label}\nt_actual={t_actual:.2f}s  pts={len(xs)}', fontsize=9)
        ax.grid(True, alpha=0.3)
        if ax is axes[0]:
            ax.legend(fontsize=7)

    plt.tight_layout()
    out_path = os.path.join(BAG_PATH, 'cloud_snapshots.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Saved → {out_path}")

# %% [markdown]
# ## 8 · Cloud Approach Animation (XY scatter sequence)
#
# Plots cloud frames at regular intervals during the obstacle encounter.
# Useful for seeing how the pole looks in the sensor at different ranges.

# %%
if HAS_CLOUD and cloud_rows:
    # Focus on the obstacle encounter window
    obs_start = diag_t[D['obs_det'] > 0.5][0]  if (D['obs_det'] > 0.5).any() else 0.0
    obs_end   = diag_t[D['obs_det'] > 0.5][-1] if (D['obs_det'] > 0.5).any() else t_end
    # Pad slightly
    w_start = max(0, obs_start - 1.0)
    w_end   = min(t_end, obs_end + 1.0)

    # Pick up to 6 evenly-spaced frames in the window
    window_mask = (cloud_ts >= w_start) & (cloud_ts <= w_end)
    window_idx  = np.where(window_mask)[0]
    if len(window_idx) < 2:
        print("Obstacle window too short for sequence plot — using full recording.")
        window_idx = np.arange(len(cloud_rows))

    n_frames = min(6, len(window_idx))
    sel_idx  = window_idx[np.linspace(0, len(window_idx)-1, n_frames, dtype=int)]

    fig, axes = plt.subplots(2, 3, figsize=(15, 9)) if n_frames > 3 else \
                plt.subplots(1, n_frames, figsize=(5*n_frames, 5))
    axes_flat = axes.flatten() if hasattr(axes, 'flatten') else axes
    fig.suptitle(f"Cloud During Obstacle Encounter — {os.path.basename(BAG_PATH)}", fontsize=11)

    for ax, idx in zip(axes_flat, sel_idx):
        _, frame_id, xs, ys, zs = cloud_rows[idx]
        t_actual = cloud_ts[idx]

        if len(xs) == 0:
            ax.text(0.5, 0.5, 'No pts', transform=ax.transAxes, ha='center'); continue

        ax.scatter(xs, ys, c=np.clip(zs, -1.2, 1.2), cmap='RdYlBu_r',
                   s=2, alpha=0.7, vmin=-1.2, vmax=1.2)
        ax.scatter(0, 0, s=100, c='white', marker='+', zorder=5, linewidths=2)
        ax.add_patch(plt.Circle((0,0), COLLISION_RADIUS, color='lime',
                                fill=False, lw=1.5, zorder=4))
        ax.add_patch(plt.Circle((0,0), MIN_CLEARANCE, color='red',
                                fill=False, lw=1.5, ls='--', zorder=4))

        # obs_d at this time from diag
        obs_d_at_t = np.interp(t_actual, diag_t, D['obs_d'])
        ax.set_title(f't={t_actual:.2f}s  obs_d={obs_d_at_t:.2f}m  pts={len(xs)}', fontsize=8)
        ax.set_xlim(-3, 3); ax.set_ylim(-3, 3)
        ax.set_aspect('equal')
        ax.set_xlabel('X fwd (m)'); ax.set_ylabel('Y left (m)')
        ax.grid(True, alpha=0.3)

    # Hide unused subplots
    for ax in axes_flat[n_frames:]:
        ax.set_visible(False)

    plt.tight_layout()
    out_path = os.path.join(BAG_PATH, 'cloud_sequence.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Saved → {out_path}")

# %% [markdown]
# ## 9 · ESTOP Event Detail

# %%
estop_mask = D['estop'] > 0.5
if not estop_mask.any():
    print("✓  No ESTOP events — drone completed the mission cleanly.")
else:
    # Cluster contiguous ESTOP ticks into events
    events = []
    in_event = False
    for i, (t, v) in enumerate(zip(diag_t, D['estop'])):
        if v > 0.5 and not in_event:
            t_ev_start = t; in_event = True
        elif v <= 0.5 and in_event:
            events.append((t_ev_start, t)); in_event = False
    if in_event:
        events.append((t_ev_start, diag_t[-1]))

    print(f"ESTOP events: {len(events)}")
    for i, (ts, te) in enumerate(events):
        mask = (diag_t >= ts - 0.1) & (diag_t <= te + 0.1)
        print(f"  Event {i+1}: t={ts:.2f}–{te:.2f}s  "
              f"min_obs_d={D['obs_d'][mask].min():.3f}m")

    n_ev   = len(events)
    fig, axes = plt.subplots(n_ev, 3, figsize=(15, 4*n_ev), squeeze=False)
    fig.suptitle(f"ESTOP Events — {os.path.basename(BAG_PATH)}", fontsize=11)
    WIN = 3.5  # seconds context around each event

    for row, (t_s, t_e) in enumerate(events):
        t_lo = max(0,    t_s - WIN)
        t_hi = min(t_end, t_e + WIN)
        m    = (diag_t >= t_lo) & (diag_t <= t_hi)

        # ── Col 0: obstacle distance ─────────────────────────────────────
        ax = axes[row][0]
        shade_status(ax, t_hi)
        ax.plot(diag_t[m], D['obs_d'][m], lw=1.5, label='obs_d')
        ax.axhline(MIN_CLEARANCE,    color='red',    ls='--', lw=1.2,
                   label=f'min_clearance ({MIN_CLEARANCE}m)')
        ax.axhline(COLLISION_RADIUS, color='purple', ls=':',  lw=1.0,
                   label=f'collision_r ({COLLISION_RADIUS}m)')
        ax.axvspan(t_s, t_e, color='red', alpha=0.25, label='ESTOP')
        ax.set_xlim(t_lo, t_hi)
        ax.set_xlabel('Time (s)'); ax.set_ylabel('obs_d (m)')
        ax.set_title(f'Event {row+1} — Obstacle Distance', fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # ── Col 1: primitive selection ────────────────────────────────────
        ax = axes[row][1]
        shade_status(ax, t_hi)
        ax.plot(diag_t[m], D['best_prim'][m], lw=1.5, color='navy', label='best_prim')
        ax.axvspan(t_s, t_e, color='red', alpha=0.25, label='ESTOP')
        ax2 = ax.twinx()
        ax2.plot(diag_t[m], D['bypass'][m], lw=1.8, color='green',
                 ls='--', drawstyle='steps-post', alpha=0.8)
        ax2.set_yticks([0,1,2]); ax2.set_yticklabels(['NONE','L','R'], color='green')
        ax.set_xlim(t_lo, t_hi)
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Best prim idx')
        ax.set_title(f'Event {row+1} — Primitive + Bypass', fontsize=9)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # ── Col 2: cloud snapshot at peak approach (closest obs_d) ────────
        ax = axes[row][2]
        if HAS_CLOUD and cloud_rows:
            t_closest = diag_t[m][D['obs_d'][m].argmin()]
            ci = int(np.argmin(np.abs(cloud_ts - t_closest)))
            _, _, xs, ys, zs = cloud_rows[ci]
            if len(xs):
                ax.scatter(xs, ys, c=np.clip(zs, -1.2, 1.2), cmap='RdYlBu_r',
                           s=2, alpha=0.7, vmin=-1.2, vmax=1.2)
            ax.scatter(0, 0, s=100, c='white', marker='+', zorder=5, linewidths=2)
            ax.add_patch(plt.Circle((0,0), COLLISION_RADIUS,
                                    color='lime', fill=False, lw=1.5))
            ax.add_patch(plt.Circle((0,0), MIN_CLEARANCE,
                                    color='red', fill=False, lw=1.5, ls='--'))
            obs_d_min = D['obs_d'][m].min()
            ax.set_title(f'Cloud at closest approach\nt={t_closest:.2f}s  obs_d={obs_d_min:.3f}m',
                         fontsize=9)
        else:
            ax.text(0.5, 0.5, 'No cloud data', transform=ax.transAxes, ha='center')
            ax.set_title(f'Event {row+1} — Cloud (N/A)', fontsize=9)
        ax.set_xlim(-3, 3); ax.set_ylim(-3, 3); ax.set_aspect('equal')
        ax.set_xlabel('X fwd (m)'); ax.set_ylabel('Y left (m)')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(BAG_PATH, 'estop_detail.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Saved → {out_path}")

# %% [markdown]
# ## 10 · Summary Statistics

# %%
print("=" * 60)
print(f"BAG : {os.path.basename(BAG_PATH)}")
print("=" * 60)
print(f"  Duration          : {diag_t[-1]:.1f} s")
print(f"  Final dist→goal   : {D['dist_goal'][-1]:.2f} m")
print(f"  Mission complete  : {mission_done}")
print()
print(f"  Min obs_d         : {D['obs_d'].min():.3f} m")
print(f"  ESTOP cycles      : {int(D['estop'].sum())}  "
      f"({100*D['estop'].mean():.0f}% of diag frames)")
print(f"  Bypass LEFT cyc   : {int((D['bypass']==1).sum())}")
print(f"  Bypass RIGHT cyc  : {int((D['bypass']==2).sum())}")
print(f"  Stall cycles      : {int(D['stalled'].sum())}")
print(f"  Orbit cycles      : {int(D['orbiting'].sum())}")
print()

from collections import Counter
cnt = Counter(s for _, s in status_rows)
print("  Status distribution:")
for s, n in cnt.most_common():
    bar = '█' * int(40 * n / len(status_rows))
    print(f"    {s:14s} {bar:40s} {n:4d} ({100*n/len(status_rows):.0f}%)")

if len(odom_rows) > 1:
    print()
    print(f"  Start pos  : ({odom_x[0]:.2f}, {odom_y[0]:.2f}, {odom_z[0]:.2f})")
    print(f"  End pos    : ({odom_x[-1]:.2f}, {odom_y[-1]:.2f}, {odom_z[-1]:.2f})")
    path_len = np.sum(np.sqrt(np.diff(odom_x)**2 + np.diff(odom_y)**2))
    print(f"  Path length: {path_len:.1f} m (XY only)")

print()
print("Saved figures:")
for fname in ['timeline.png', 'trajectory.png', 'cloud_snapshots.png',
              'cloud_sequence.png', 'estop_detail.png']:
    fp = os.path.join(BAG_PATH, fname)
    if os.path.exists(fp):
        print(f"  {fp}")
