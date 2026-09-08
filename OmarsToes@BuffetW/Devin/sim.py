import tkinter as tk
from tkinter import ttk
import math, time, json, bisect, random
from pathlib import Path

from PIL import Image, ImageTk

rdir = Path(__file__).resolve().parent.parent.parent / "src" / "main" / "deploy" / "pathplanner"
if not rdir.exists():
    rdir = Path(__file__).resolve().parent / "src" / "main" / "deploy" / "pathplanner"
if not rdir.exists():
    rdir = Path("src/main/deploy/pathplanner")
sf = rdir / "settings.json"
adir = rdir / "autos"
pdir = rdir / "paths"

OBS_FILE = Path(__file__).resolve().parent / "field_obstacles.json"
BG_FILE = Path(__file__).resolve().parent / "field_background.png"
CALIB_FILE = Path(__file__).resolve().parent / "field_config.json"

with open(sf) as f:
    cfg = json.load(f)

RW = float(cfg.get("robotWidth", 0.864))
RL = float(cfg.get("robotLength", 0.851))
MV = float(cfg.get("defaultMaxVel", 5.25))
MA = float(cfg.get("defaultMaxAccel", 5.2))
MAW = math.radians(float(cfg.get("defaultMaxAngVel", 540.0)))
MAA = math.radians(float(cfg.get("defaultMaxAngAccel", 720.0)))
COF = float(cfg.get("wheelCOF", 2.255))
G = 9.80665

IX, IY, IW, IL = 0.5271, 0.0, 0.635, 0.203
try:
    ft = cfg.get("robotFeatures", [])
    if ft:
        fd = json.loads(ft[0])
        IX = float(fd["data"]["center"]["x"])
        IY = float(fd["data"]["center"]["y"])
        IW = float(fd["data"]["size"]["width"])
        IL = float(fd["data"]["size"]["length"])
except Exception:
    pass

R_RPM = 1000.0
R_RAD = (2.0 * 0.0254) / 2.0
R_SPD = math.pi * (2.0 * R_RAD) * (R_RPM / 60.0)

SX = -0.38
SY = 0.0

DEFAULT_CALIB = {
    "FL": 16.532,
    "FW": 8.001,
    "img_crop": [122, 19, 824, 365],
    "BHUB": [4.616, 4.051],
    "RHUB": [11.801, 4.052],
    "BHUB_RAD": 0.574,
    "RHUB_RAD": 0.60,
    "NZ": [7.399, 1.917, 9.082, 6.233],
    "BDP": [0.117, 5.55, 0.669, 6.367],
    "RDP": [15.846, 1.434, 16.532, 2.501]
}

def load_calib():
    if CALIB_FILE.exists():
        try:
            with open(CALIB_FILE) as f:
                data = json.load(f)
                res = dict(DEFAULT_CALIB)
                res.update(data)
                return res
        except Exception:
            pass
    return dict(DEFAULT_CALIB)

def save_calib(c_dict):
    try:
        with open(CALIB_FILE, "w") as f:
            json.dump(c_dict, f, indent=2)
    except Exception as ex:
        print("Error saving calib:", ex)

CALIB = load_calib()
FL = float(CALIB["FL"])
FW = float(CALIB["FW"])
BHUB = tuple(CALIB["BHUB"])
RHUB = tuple(CALIB["RHUB"])
BHUB_RAD = float(CALIB.get("BHUB_RAD", 0.574))
RHUB_RAD = float(CALIB.get("RHUB_RAD", 0.60))

BD = 5.91 * 0.0254
BR = BD / 2.0
MAX_HP = 50

NZ_X0, NZ_Y0, NZ_X1, NZ_Y1 = CALIB["NZ"]
BDP_X0, BDP_Y0, BDP_X1, BDP_Y1 = CALIB["BDP"]
RDP_X0, RDP_Y0, RDP_X1, RDP_Y1 = CALIB["RDP"]

DEFAULT_OBSTACLES = [
    {"name": "Blue Bottom Bump", "type": "bump", "x0": 4.0, "y0": 1.47, "x1": 5.19, "y1": 3.45},
    {"name": "Red Bottom Bump", "type": "bump", "x0": 11.25, "y0": 1.4, "x1": 12.43, "y1": 3.46},
    {"name": "Blue Hub Barrier", "type": "barrier", "x0": 3.96, "y0": 3.47, "x1": 5.22, "y1": 4.7},
    {"name": "Red Hub Barrier", "type": "barrier", "x0": 11.29, "y0": 3.47, "x1": 12.45, "y1": 4.67},
    {"name": "Blue Top Bump", "type": "bump", "x0": 3.95, "y0": 4.72, "x1": 5.18, "y1": 6.72},
    {"name": "Red Top Bump", "type": "bump", "x0": 11.27, "y0": 4.65, "x1": 12.42, "y1": 6.68}
]

def load_obstacles():
    if OBS_FILE.exists():
        try:
            with open(OBS_FILE) as f:
                data = json.load(f)
                if isinstance(data, list):
                    return data
        except Exception:
            pass
    return [dict(o) for o in DEFAULT_OBSTACLES]

def save_obstacles(obs_list):
    try:
        with open(OBS_FILE, "w") as f:
            json.dump(obs_list, f, indent=2)
    except Exception as ex:
        print("Error saving obstacles:", ex)

Z3_T = 6.0
Z2_T = 3.2
Z12_H = 3.35
Z21_H = 3.05

def get_rpm(d, cur=2200.0):
    if d >= Z3_T:
        return 3750.0
    if cur == 3750.0:
        return 2750.0 if d < 5.8 else 3750.0
    elif cur == 2200.0:
        return 2750.0 if d >= Z12_H else 2200.0
    else:
        return 2200.0 if d < Z21_H else 2750.0

def calc_pitch(d, v=15.0):
    y = 55.0 * 0.0254
    v2 = v * v
    term = v2 * v2 - G * (G * d * d + 2.0 * y * v2)
    if term < 0:
        return math.radians(45.0)
    root = math.sqrt(term)
    return math.atan2(v2 + root, G * d)

def bz(p0, p1, p2, p3, t):
    u = 1.0 - t
    tt = t * t
    uu = u * u
    x = uu * u * p0[0] + 3.0 * uu * t * p1[0] + 3.0 * u * tt * p2[0] + tt * t * p3[0]
    y = uu * u * p0[1] + 3.0 * uu * t * p1[1] + 3.0 * u * tt * p2[1] + tt * t * p3[1]
    dx = 3.0 * uu * (p1[0] - p0[0]) + 6.0 * u * t * (p2[0] - p1[0]) + 3.0 * tt * (p3[0] - p2[0])
    dy = 3.0 * uu * (p1[1] - p0[1]) + 6.0 * u * t * (p2[1] - p1[1]) + 3.0 * tt * (p3[1] - p2[1])
    ddx = 6.0 * u * (p2[0] - 2.0 * p1[0] + p0[0]) + 6.0 * t * (p3[0] - 2.0 * p2[0] + p1[0])
    ddy = 6.0 * u * (p2[1] - 2.0 * p1[1] + p0[1]) + 6.0 * t * (p3[1] - 2.0 * p2[1] + p1[1])
    return x, y, dx, dy, ddx, ddy

def n_ang(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def a_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi

class Traj:
    def __init__(self, fn, fp, mx=False, my=False):
        self.fn = fn
        self.fp = fp
        self.mx = mx
        self.my = my
        self.pts = []
        self.tms = []
        self.tot_t = 0.0
        self.tot_d = 0.0
        self.v0 = 0.0
        self.v1 = 0.0
        self.p0 = (0.0, 0.0, 0.0)
        self.p1 = (0.0, 0.0, 0.0)
        self.bake()

    def bake(self):
        with open(self.fp) as f:
            d = json.load(f)
        wps = d.get("waypoints", [])
        if not wps:
            return
        iss = d.get("idealStartingState", {})
        ges = d.get("goalEndState", {})
        r0 = float(iss.get("rotation", 0.0))
        r1 = float(ges.get("rotation", 0.0))
        self.v0 = float(iss.get("velocity", 0.0))
        self.v1 = float(ges.get("velocity", 0.0))
        rts = d.get("rotationTargets", [])
        gc = d.get("globalConstraints", {})
        mv = float(gc.get("maxVelocity", MV))
        ma = float(gc.get("maxAcceleration", MA))

        raw = []
        seg_n = 50
        for i in range(len(wps) - 1):
            w0, w1 = wps[i], wps[i+1]
            p0 = (w0["anchor"]["x"], w0["anchor"]["y"])
            p1 = (w0["nextControl"]["x"], w0["nextControl"]["y"]) if w0["nextControl"] else p0
            p2 = (w1["prevControl"]["x"], w1["prevControl"]["y"]) if w1["prevControl"] else (w1["anchor"]["x"], w1["anchor"]["y"])
            p3 = (w1["anchor"]["x"], w1["anchor"]["y"])
            for s in range(seg_n):
                t = s / float(seg_n)
                x, y, dx, dy, ddx, ddy = bz(p0, p1, p2, p3, t)
                raw.append((x, y, dx, dy, ddx, ddy, i + t))
        last_w = wps[-1]
        raw.append((last_w["anchor"]["x"], last_w["anchor"]["y"], 0.0, 0.0, 0.0, 0.0, float(len(wps) - 1)))

        cum = [0.0]
        for k in range(1, len(raw)):
            cum.append(cum[-1] + math.hypot(raw[k][0] - raw[k-1][0], raw[k][1] - raw[k-1][1]))
        self.tot_d = cum[-1]
        N = len(raw)

        def rot_at(rp, dist):
            if not rts:
                frac = dist / max(0.01, self.tot_d)
                diff = (r1 - r0 + 180.0) % 360.0 - 180.0
                return r0 + diff * frac
            if rp <= rts[0]["waypointRelativePos"]:
                frac = max(0.0, min(1.0, rp / max(0.01, rts[0]["waypointRelativePos"])))
                diff = (rts[0]["rotationDegrees"] - r0 + 180.0) % 360.0 - 180.0
                return r0 + diff * frac
            elif rp >= rts[-1]["waypointRelativePos"]:
                denom = max(0.01, len(wps) - 1 - rts[-1]["waypointRelativePos"])
                frac = max(0.0, min(1.0, (rp - rts[-1]["waypointRelativePos"]) / denom))
                diff = (r1 - rts[-1]["rotationDegrees"] + 180.0) % 360.0 - 180.0
                return rts[-1]["rotationDegrees"] + diff * frac
            else:
                for k in range(len(rts) - 1):
                    if rts[k]["waypointRelativePos"] <= rp <= rts[k+1]["waypointRelativePos"]:
                        span = max(0.01, rts[k+1]["waypointRelativePos"] - rts[k]["waypointRelativePos"])
                        frac = max(0.0, min(1.0, (rp - rts[k]["waypointRelativePos"]) / span))
                        diff = (rts[k+1]["rotationDegrees"] - rts[k]["rotationDegrees"] + 180.0) % 360.0 - 180.0
                        return rts[k]["rotationDegrees"] + diff * frac
                return r1

        ac = COF * G
        vlim = []
        for k in range(N):
            x, y, dx, dy, ddx, ddy, rp = raw[k]
            sp2 = dx * dx + dy * dy
            curv = abs(dx * ddy - dy * ddx) / (sp2 ** 1.5) if sp2 > 1e-6 else 0.0
            vc = math.sqrt(ac / curv) if curv > 1e-4 else mv
            vlim.append(min(mv, max(1.0, vc)))

        vp = list(vlim)
        vp[0] = min(vp[0], self.v0) if self.v0 > 0.0 else min(vp[0], 0.2)
        vp[-1] = min(vp[-1], self.v1) if self.v1 > 0.0 else min(vp[-1], 0.0)

        for k in range(1, N):
            ds = cum[k] - cum[k-1]
            vp[k] = min(vp[k], math.sqrt(vp[k-1]**2 + 2.0 * ma * ds))

        for k in range(N - 2, -1, -1):
            ds = cum[k+1] - cum[k]
            vp[k] = min(vp[k], math.sqrt(vp[k+1]**2 + 2.0 * ma * ds))

        tms = [0.0]
        for k in range(1, N):
            ds = cum[k] - cum[k-1]
            v_mid = (vp[k] + vp[k-1]) / 2.0
            tms.append(tms[-1] + ds / max(0.15, v_mid))
        self.tot_t = tms[-1]
        self.tms = tms

        pts = []
        for k in range(N):
            x, y, dx, dy, ddx, ddy, rp = raw[k]
            s = cum[k]
            t = tms[k]
            v = vp[k]
            if math.hypot(dx, dy) > 1e-4:
                hd = math.atan2(dy, dx)
            elif k < N - 1:
                ndx = raw[k+1][0] - x
                ndy = raw[k+1][1] - y
                hd = math.atan2(ndy, ndx) if math.hypot(ndx, ndy) > 1e-4 else 0.0
            else:
                hd = pts[-1]["hd"] if pts else 0.0
            rot = math.radians(rot_at(rp, s))
            vx = v * math.cos(hd)
            vy = v * math.sin(hd)
            if self.mx:
                x = FL - x
                vx = -vx
                hd = math.pi - hd
                rot = math.pi - rot
            if self.my:
                y = FW - y
                vy = -vy
                hd = -hd
                rot = -rot
            pts.append({
                "t": t, "s": s, "x": x, "y": y, "v": v, "vx": vx, "vy": vy,
                "hd": n_ang(hd), "rot": n_ang(rot), "w": 0.0
            })

        for k in range(len(pts)):
            if k < len(pts) - 1:
                dt_k = max(0.005, pts[k+1]["t"] - pts[k]["t"])
                pts[k]["w"] = a_diff(pts[k+1]["rot"], pts[k]["rot"]) / dt_k
            else:
                pts[k]["w"] = pts[k-1]["w"] if k > 0 else 0.0

        self.pts = pts
        if pts:
            self.p0 = (pts[0]["x"], pts[0]["y"], pts[0]["rot"])
            self.p1 = (pts[-1]["x"], pts[-1]["y"], pts[-1]["rot"])

    def sample(self, t):
        if not self.pts:
            return {"x": 0.0, "y": 0.0, "rot": 0.0, "v": 0.0, "vx": 0.0, "vy": 0.0, "w": 0.0, "hd": 0.0}
        if t <= 0.0:
            return self.pts[0]
        if t >= self.tot_t:
            return self.pts[-1]
        k = bisect.bisect_right(self.tms, t) - 1
        k = max(0, min(len(self.pts) - 2, k))
        t0, t1 = self.tms[k], self.tms[k+1]
        a = (t - t0) / max(1e-5, t1 - t0)
        p0, p1 = self.pts[k], self.pts[k+1]
        x = p0["x"] + a * (p1["x"] - p0["x"])
        y = p0["y"] + a * (p1["y"] - p0["y"])
        v = p0["v"] + a * (p1["v"] - p0["v"])
        vx = p0["vx"] + a * (p1["vx"] - p0["vx"])
        vy = p0["vy"] + a * (p1["vy"] - p0["vy"])
        rot = n_ang(p0["rot"] + a * a_diff(p1["rot"], p0["rot"]))
        w = p0["w"] + a * (p1["w"] - p0["w"])
        hd = n_ang(p0["hd"] + a * a_diff(p1["hd"], p0["hd"]))
        return {"x": x, "y": y, "v": v, "vx": vx, "vy": vy, "rot": rot, "w": w, "hd": hd}

class B:
    def __init__(self, x, y, bid, r=BR):
        self.x = x
        self.y = y
        self.id = bid
        self.r = r
        self.vx = 0.0
        self.vy = 0.0
        self.iid = None
        self.dep = 0.0

class P:
    def __init__(self, sx, sy, tx, ty, t_tot=0.6, h=1.5):
        self.sx = sx
        self.sy = sy
        self.tx = tx
        self.ty = ty
        self.t_tot = t_tot
        self.t = 0.0
        self.h = h
        self.done = False
        self.cx = sx
        self.cy = sy
        self.ah = 0.0

    def update(self, dt):
        self.t += dt
        s = min(1.0, self.t / max(0.01, self.t_tot))
        self.cx = self.sx + s * (self.tx - self.sx)
        self.cy = self.sy + s * (self.ty - self.sy)
        self.ah = 4.0 * self.h * s * (1.0 - s)
        if self.t >= self.t_tot:
            self.done = True
            return True
        return False

class F:
    def __init__(self, al="Red"):
        self.al = al
        self.balls = []
        self.projs = []
        self.scored = 0
        self.obstacles = load_obstacles()
        self.reset_balls()

    def reset_balls(self):
        self.balls.clear()
        self.projs.clear()
        self.scored = 0
        bid = 0
        dx = (NZ_X1 - NZ_X0) / 11.0
        dy = (NZ_Y1 - NZ_Y0) / 29.0
        for c in range(12):
            for r in range(30):
                self.balls.append(B(NZ_X0 + c * dx, NZ_Y0 + r * dy, bid))
                bid += 1
        bd_dx = (BDP_X1 - BDP_X0) / 3.0
        bd_dy = (BDP_Y1 - BDP_Y0) / 5.0
        for c in range(4):
            for r in range(6):
                self.balls.append(B(BDP_X0 + 0.04 + c * bd_dx * 0.85, BDP_Y0 + 0.04 + r * bd_dy * 0.85, bid))
                bid += 1
        rd_dx = (RDP_X1 - RDP_X0) / 3.0
        rd_dy = (RDP_Y1 - RDP_Y0) / 5.0
        for c in range(4):
            for r in range(6):
                self.balls.append(B(RDP_X0 + 0.04 + c * rd_dx * 0.85, RDP_Y0 + 0.04 + r * rd_dy * 0.85, bid))
                bid += 1

    def tgt_hub(self, al):
        return RHUB if al == "Red" else BHUB

    def col_obstacles(self, dt):
        for b in self.balls:
            for obs in self.obstacles:
                x0, y0 = min(obs["x0"], obs["x1"]), min(obs["y0"], obs["y1"])
                x1, y1 = max(obs["x0"], obs["x1"]), max(obs["y0"], obs["y1"])
                otype = obs.get("type", "barrier")

                if otype == "bump":
                    if x0 <= b.x <= x1 and y0 <= b.y <= y1:
                        # Ball crosses speed bump: resistance and minor deflect
                        b.vx *= max(0.0, 1.0 - 10.0 * dt)
                        b.vy *= max(0.0, 1.0 - 10.0 * dt)
                else:
                    # Solid barrier: closest point on AABB to circle
                    cx = max(x0, min(x1, b.x))
                    cy = max(y0, min(y1, b.y))
                    dx = b.x - cx
                    dy = b.y - cy
                    d2 = dx * dx + dy * dy
                    if d2 < BR * BR:
                        d = math.sqrt(d2)
                        if d > 1e-4:
                            nx, ny = dx / d, dy / d
                            pen = BR - d
                        else:
                            # Inside box center: push toward closest edge
                            dl, dr = b.x - x0, x1 - b.x
                            db, dt_e = b.y - y0, y1 - b.y
                            m = min(dl, dr, db, dt_e)
                            if m == dl: nx, ny, pen = -1.0, 0.0, dl + BR
                            elif m == dr: nx, ny, pen = 1.0, 0.0, dr + BR
                            elif m == db: nx, ny, pen = 0.0, -1.0, db + BR
                            else: nx, ny, pen = 0.0, 1.0, dt_e + BR
                        b.x += nx * pen
                        b.y += ny * pen
                        vn = b.vx * nx + b.vy * ny
                        if vn < 0:
                            b.vx -= (1.6) * vn * nx
                            b.vy -= (1.6) * vn * ny

    def col_balls(self):
        csize = 0.20
        grid = {}
        for b in self.balls:
            gx, gy = int(b.x / csize), int(b.y / csize)
            grid.setdefault((gx, gy), []).append(b)
        rad2 = BR * 2.0
        rad2_sq = rad2 * rad2
        for _ in range(2):
            for (gx, gy), b_list in grid.items():
                nbrs = []
                for ox in (-1, 0, 1):
                    for oy in (-1, 0, 1):
                        nbrs.extend(grid.get((gx + ox, gy + oy), []))
                for i in range(len(b_list)):
                    b1 = b_list[i]
                    for b2 in nbrs:
                        if b1.id >= b2.id:
                            continue
                        dx = b2.x - b1.x
                        dy = b2.y - b1.y
                        d2 = dx * dx + dy * dy
                        if 1e-6 < d2 < rad2_sq:
                            d = math.sqrt(d2)
                            ol = rad2 - d
                            nx, ny = dx / d, dy / d
                            b1.x -= nx * ol * 0.5
                            b1.y -= ny * ol * 0.5
                            b2.x += nx * ol * 0.5
                            b2.y += ny * ol * 0.5
                            dvx = b1.vx - b2.vx
                            dvy = b1.vy - b2.vy
                            dot = dvx * nx + dvy * ny
                            if dot > 0:
                                b1.vx -= dot * nx * 0.7
                                b1.vy -= dot * ny * 0.7
                                b2.vx += dot * nx * 0.7
                                b2.vy += dot * ny * 0.7

    def step(self, dt):
        self.col_balls()
        self.col_obstacles(dt)
        fric = max(0.0, 1.0 - 5.0 * dt)
        for b in self.balls:
            b.x += b.vx * dt
            b.y += b.vy * dt
            b.vx *= fric
            b.vy *= fric
            if math.hypot(b.vx, b.vy) < 0.05:
                b.vx = 0.0
                b.vy = 0.0
            b.x = max(BR, min(FL - BR, b.x))
            b.y = max(BR, min(FW - BR, b.y))
        self.col_obstacles(dt)

        rem = []
        for p in self.projs:
            if p.update(dt):
                self.scored += 1
            else:
                rem.append(p)
        self.projs = rem

class R:
    def __init__(self, x=0.0, y=0.0, th=0.0, rid="R0", nm="Robot", al="Red", col="#ff5555"):
        self.x = x
        self.y = y
        self.th = th
        self.id = rid
        self.name = nm
        self.al = al
        self.col = col
        self.w = RW
        self.l = RL
        self.start_x = x
        self.start_y = y
        self.start_th = th
        self.vx = 0.0
        self.vy = 0.0
        self.w_rot = 0.0
        self.hp = 0
        self.intake = False
        self.shooting = False
        self.rpm = 2200.0
        self.tgt_rpm = 2200.0
        self.shots = 0
        self.cd = 0.0

    def corners(self):
        hw, hl = self.w / 2.0, self.l / 2.0
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)
        res = []
        for ox, oy in [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]:
            res.append((self.x + ox * cos_t - oy * sin_t, self.y + ox * sin_t + oy * cos_t))
        return res

    def intake_poly(self):
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)
        hw, hl = IW / 2.0, IL / 2.0
        res = []
        for ox, oy in [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]:
            lx, ly = IX + ox, IY + oy
            res.append((self.x + lx * cos_t - ly * sin_t, self.y + lx * sin_t + ly * cos_t))
        return res

    def intake_front(self):
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)
        hw, hl = IW / 2.0, IL / 2.0
        fx = IX + hl
        p1 = (self.x + fx * cos_t - (-hw) * sin_t, self.y + fx * sin_t + (-hw) * cos_t)
        p2 = (self.x + fx * cos_t - hw * sin_t, self.y + fx * sin_t + hw * cos_t)
        return p1, p2

    def shooter_pos(self):
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)
        return (self.x + SX * cos_t - SY * sin_t, self.y + SX * sin_t + SY * cos_t)

    def shoot(self, fld, thub):
        if self.hp <= 0:
            return False
        sp = self.shooter_pos()
        tx, ty = thub
        tx += random.uniform(-0.12, 0.12)
        ty += random.uniform(-0.12, 0.12)
        dist = math.hypot(tx - sp[0], ty - sp[1])
        fld.projs.append(P(sp[0], sp[1], tx, ty, t_tot=max(0.35, dist / 11.5), h=1.4))
        self.hp -= 1
        self.shots += 1
        return True

    def update_intake(self, fld, dt):
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)
        hw, hl = self.w / 2.0, self.l / 2.0
        ihw = IW / 2.0
        rem_b = []
        for b in fld.balls:
            if b.iid is not None and b.iid != self.id:
                rem_b.append(b)
                continue
            dx = b.x - self.x
            dy = b.y - self.y
            rx = dx * cos_t + dy * sin_t
            ry = -dx * sin_t + dy * cos_t

            in_intake = (self.intake and
                         (hl - 0.05 <= rx <= IX + IL / 2.0 + BR) and
                         (abs(ry - IY) <= ihw + BR * 0.5))

            if in_intake:
                if self.hp < MAX_HP:
                    b.iid = self.id
                    prx = rx - R_SPD * dt
                    b.dep += R_SPD * dt
                    if b.dep >= 0.10 or prx <= (hl + 0.02):
                        self.hp += 1
                        b.iid = None
                        continue
                    b.x = self.x + prx * cos_t - ry * sin_t
                    b.y = self.y + prx * sin_t + ry * cos_t
                    rem_b.append(b)
                    continue
                else:
                    if b.iid == self.id:
                        b.iid = None
                        b.dep = 0.0

            if b.iid == self.id and not in_intake:
                b.iid = None
                b.dep = 0.0

            if abs(rx) < hl + BR and abs(ry) < hw + BR:
                ox = (hl + BR) - abs(rx)
                oy = (hw + BR) - abs(ry)
                if ox < oy:
                    rx = math.copysign(hl + BR + 0.005, rx)
                else:
                    ry = math.copysign(hw + BR + 0.005, ry)
                b.x = self.x + rx * cos_t - ry * sin_t
                b.y = self.y + rx * sin_t + ry * cos_t
                b.vx = self.vx - self.w_rot * dy * 0.5
                b.vy = self.vy + self.w_rot * dx * 0.5
            rem_b.append(b)
        fld.balls = rem_b

def col_robs(robs):
    min_d = 0.88
    for i in range(len(robs)):
        for j in range(i + 1, len(robs)):
            r1, r2 = robs[i], robs[j]
            dx = r2.x - r1.x
            dy = r2.y - r1.y
            d = math.hypot(dx, dy)
            if d < min_d:
                if d < 1e-4:
                    dx, dy, d = 0.1, 0.0, 0.1
                ol = min_d - d
                nx, ny = dx / d, dy / d
                r1.x -= nx * ol * 0.5
                r1.y -= ny * ol * 0.5
                r2.x += nx * ol * 0.5
                r2.y += ny * ol * 0.5
                rvx = r1.vx - r2.vx
                rvy = r1.vy - r2.vy
                dot = rvx * nx + rvy * ny
                if dot > 0:
                    imp = dot * 0.35
                    r1.vx -= imp * nx
                    r1.vy -= imp * ny
                    r2.vx += imp * nx
                    r2.vy += imp * ny

    # Robot vs Barrier Obstacles
    rad_r = 0.44  # effective chassis radius
    for r in robs:
        for obs in getattr(r, "_obstacles", []):
            if obs.get("type", "barrier") != "barrier":
                continue
            x0, y0 = min(obs["x0"], obs["x1"]), min(obs["y0"], obs["y1"])
            x1, y1 = max(obs["x0"], obs["x1"]), max(obs["y0"], obs["y1"])
            cx = max(x0, min(x1, r.x))
            cy = max(y0, min(y1, r.y))
            dx = r.x - cx
            dy = r.y - cy
            d2 = dx * dx + dy * dy
            if d2 < rad_r * rad_r:
                d = math.sqrt(d2)
                if d > 1e-4:
                    nx, ny = dx / d, dy / d
                    pen = rad_r - d
                else:
                    dl, dr = r.x - x0, x1 - r.x
                    db, dt_e = r.y - y0, y1 - r.y
                    m = min(dl, dr, db, dt_e)
                    if m == dl: nx, ny, pen = -1.0, 0.0, dl + rad_r
                    elif m == dr: nx, ny, pen = 1.0, 0.0, dr + rad_r
                    elif m == db: nx, ny, pen = 0.0, -1.0, db + rad_r
                    else: nx, ny, pen = 0.0, 1.0, dt_e + rad_r
                r.x += nx * pen
                r.y += ny * pen
                vn = r.vx * nx + r.vy * ny
                if vn < 0:
                    r.vx -= vn * nx
                    r.vy -= vn * ny

class Node:
    def __init__(self, t, d=None):
        self.t = t
        self.d = d or {}
        self.ch = []

class E:
    def __init__(self, r, fld):
        self.r = r
        self.fld = fld
        self.cache = {}
        self.root = None
        self.active = False
        self.exec = None
        self.cur_cmd = "Idle"
        self.dt = 0.02

    def load_auto(self, aname, mx=False, my=False):
        self.cache.clear()
        self.active = False
        self.cur_cmd = "Idle"
        if aname == "Stay Still (Idle)" or not aname:
            self.root = Node("wait", {"waitTime": 9999.0})
            self.exec = self.build_exec(self.root)
            return
        afp = adir / aname
        if not afp.exists():
            self.root = Node("wait", {"waitTime": 9999.0})
            self.exec = self.build_exec(self.root)
            return
        with open(afp) as f:
            data = json.load(f)

        def get_paths(node):
            if node.get("type") == "path":
                pn = node.get("data", {}).get("pathName")
                if pn and pn not in self.cache:
                    pfp = pdir / f"{pn}.path"
                    if pfp.exists():
                        self.cache[pn] = Traj(pn, pfp, mx=mx, my=my)
            for c in node.get("data", {}).get("commands", []):
                get_paths(c)

        cmd_root = data.get("command", {})
        get_paths(cmd_root)

        def parse_n(raw):
            n = Node(raw.get("type", "noop"), raw.get("data", {}))
            for c in raw.get("data", {}).get("commands", []):
                n.ch.append(parse_n(c))
            return n

        self.root = parse_n(cmd_root)
        self.exec = self.build_exec(self.root)

        # Extract first path in execution sequence to get starting pose
        first_pname = None
        def find_first_path(n):
            nonlocal first_pname
            if first_pname is not None:
                return
            if n.t == "path":
                first_pname = n.d.get("pathName")
                return
            for child in n.ch:
                find_first_path(child)
                if first_pname is not None:
                    return

        find_first_path(self.root)
        if first_pname and first_pname in self.cache:
            tr = self.cache[first_pname]
            if tr.pts:
                p0 = tr.pts[0]
                self.r.start_x = p0["x"]
                self.r.start_y = p0["y"]
                self.r.start_th = p0["rot"]
        self.reset_robot_pose()

    def reset_robot_pose(self):
        self.r.vx = 0.0
        self.r.vy = 0.0
        self.r.w_rot = 0.0
        self.r.intake = False
        self.r.shooting = False
        self.r.hp = 0
        if hasattr(self.r, "start_x") and self.r.start_x is not None:
            self.r.x = self.r.start_x
            self.r.y = self.r.start_y
            self.r.th = self.r.start_th
        self.cur_cmd = "Idle"

    def start(self):
        if self.root:
            self.exec = self.build_exec(self.root)
            self.active = True
            self.cur_cmd = "Running Auto"

    def stop(self):
        self.active = False
        self.r.vx = 0.0
        self.r.vy = 0.0
        self.r.w_rot = 0.0
        self.r.intake = False
        self.r.shooting = False
        self.cur_cmd = "Idle"

    def update(self, dt):
        self.dt = dt
        if self.active and self.exec:
            if self.exec["step"]():
                self.active = False
                self.r.vx = 0.0
                self.r.vy = 0.0
                self.r.w_rot = 0.0
                self.r.intake = False
                self.r.shooting = False
                self.cur_cmd = "Completed"

    def build_exec(self, n):
        t = n.t
        d = n.d
        if t == "sequential":
            cx = [self.build_exec(c) for c in n.ch]
            st = {"idx": 0}
            def step_seq():
                while st["idx"] < len(cx):
                    if cx[st["idx"]]["step"]():
                        st["idx"] += 1
                    else:
                        return False
                return True
            return {"type": "seq", "step": step_seq}

        elif t == "parallel":
            cx = [self.build_exec(c) for c in n.ch]
            def step_par():
                pdone = False
                for i, c in enumerate(cx):
                    done = c["step"]()
                    if i == 0 and done:
                        pdone = True
                return pdone
            return {"type": "par", "step": step_par}

        elif t == "race":
            cx = [self.build_exec(c) for c in n.ch]
            def step_race():
                for c in cx:
                    if c["step"]():
                        return True
                return False
            return {"type": "race", "step": step_race}

        elif t == "wait":
            wt = float(d.get("waitTime", 1.0))
            st = {"t": 0.0}
            def step_wait():
                st["t"] += self.dt
                return st["t"] >= wt
            return {"type": "wait", "step": step_wait}

        elif t == "path":
            pn = d.get("pathName")
            tr = self.cache.get(pn)
            dur = tr.tot_t if tr else 1.0
            st = {"t": 0.0}
            def step_pth():
                st["t"] += self.dt
                if tr:
                    s = tr.sample(st["t"])
                    tx, ty, trot = s["x"], s["y"], s["rot"]
                    fvx, fvy, fw = s["vx"], s["vy"], s["w"]
                    ex, ey = tx - self.r.x, ty - self.r.y
                    cvx = fvx + 10.0 * ex
                    cvy = fvy + 10.0 * ey
                    spd = math.hypot(cvx, cvy)
                    max_v = max(5.36, MV * 1.15)
                    if spd > max_v:
                        sc = max_v / spd
                        cvx *= sc
                        cvy *= sc
                    dth = a_diff(trot, self.r.th)
                    cw = max(-MAW, min(MAW, fw + 10.0 * dth))
                    self.r.vx = cvx
                    self.r.vy = cvy
                    self.r.w_rot = cw
                    self.r.x += self.r.vx * self.dt
                    self.r.y += self.r.vy * self.dt
                    self.r.th = n_ang(self.r.th + self.r.w_rot * self.dt)
                self.cur_cmd = f"Path: {pn}"
                if st["t"] >= dur:
                    if tr and tr.v1 > 0.5:
                        return True
                    err = math.hypot(s["x"] - self.r.x, s["y"] - self.r.y) if tr else 0.0
                    if err < 0.15 or st["t"] >= dur + 0.35:
                        self.r.vx = 0.0
                        self.r.vy = 0.0
                        self.r.w_rot = 0.0
                        return True
                return False
            return {"type": "path", "step": step_pth}

        elif t == "named":
            nm = d.get("name")
            return self.build_named(nm)

        return {"type": "noop", "step": lambda: True}

    def build_named(self, nm):
        if nm == "ShootAutoAim":
            st = {"t": 0.0, "shots": 0, "cd": 0.0}
            def step_shoot():
                st["t"] += self.dt
                st["cd"] -= self.dt
                thub = self.fld.tgt_hub(self.r.al)
                sp = self.r.shooter_pos()
                dist = math.hypot(thub[0] - sp[0], thub[1] - sp[1])
                req_rpm = get_rpm(dist, self.r.rpm)
                self.r.tgt_rpm = req_rpm
                if self.r.rpm < self.r.tgt_rpm:
                    self.r.rpm = min(self.r.tgt_rpm, self.r.rpm + 4000.0 * self.dt)
                else:
                    self.r.rpm = max(self.r.tgt_rpm, self.r.rpm - 4000.0 * self.dt)

                req_ang = n_ang(math.atan2(thub[1] - sp[1], thub[0] - sp[0]) + math.pi)
                dth = a_diff(req_ang, self.r.th)
                self.r.w_rot = max(-MAW, min(MAW, 8.0 * dth))
                self.r.th = n_ang(self.r.th + self.r.w_rot * self.dt)
                self.r.vx = 0.0
                self.r.vy = 0.0

                rpm_ok = abs(self.r.rpm - self.r.tgt_rpm) <= 75.0
                th_ok = abs(dth) < math.radians(3.0)
                locked = rpm_ok and th_ok

                if locked:
                    self.r.shooting = True
                    self.cur_cmd = f"AimBot [LOCKED]: {int(self.r.rpm)} RPM"
                    if st["cd"] <= 0.0 and self.r.hp > 0:
                        if self.r.shoot(self.fld, thub):
                            st["shots"] += 1
                            st["cd"] = 0.10
                else:
                    self.r.shooting = False
                    self.cur_cmd = f"AimBot [ALIGNING]: {int(self.r.rpm)} RPM"

                if (self.r.hp == 0 and st["shots"] > 0) or st["t"] >= 3.0:
                    self.r.shooting = False
                    return True
                return False
            return {"type": "named", "step": step_shoot}

        elif nm == "Intake":
            def step_intake():
                self.r.intake = True
                self.cur_cmd = "Intake [ACTIVE]"
                return True
            return {"type": "named", "step": step_intake}

        return {"type": "noop", "step": lambda: True}

class Sim:
    def __init__(self, root):
        self.root = root
        self.root.title("FRC 2026 Simulation")
        self.root.geometry("1400x820")
        self.root.configure(bg="#1e1e24")

        self.fld = F("Red")
        self.robs = [
            R(4.52, 7.36, 0.0, "B1", "Blue Top (L)", "Blue", "#8be9fd"),
            R(4.52, 0.64, 0.0, "B2", "Blue Bottom (R)", "Blue", "#50fa7b"),
            R(FL - 4.52, 7.36, math.pi, "R1", "Red Top (L)", "Red", "#ff5555"),
            R(FL - 4.52, 0.64, math.pi, "R2", "Red Bottom (R)", "Red", "#ffb86c")
        ]
        self.engs = [E(r, self.fld) for r in self.robs]
        self.cfgs = [
            {"mx": False, "my": False, "def": "L-Double-Tap-Trench-Depot.auto"},
            {"mx": False, "my": False, "def": "R-Double-Tap-Trench.auto"},
            {"mx": True, "my": False, "def": "L-Double-Tap-Trench-Depot.auto"},
            {"mx": True, "my": False, "def": "R-Double-Tap-Trench.auto"},
        ]

        self.spd = 1.0
        self.zm = 60.0
        self.ox = 35.0
        self.oy = 35.0
        self.b_items = {}
        self.init_done = False

        # Collision Builder State
        self.builder_mode = False
        self.builder_tool = "select"
        self.drag_start = None
        self.drag_target = None
        self.drag_handle = None
        self.drag_init_geom = None
        self.preview_rect_id = None
        self.bg_img_raw = None
        self.bg_img_tk = None
        self.load_bg_img()

        # Wire obstacles to robots for collision
        for r in self.robs:
            r._obstacles = self.fld.obstacles

        self.ui()
        self.init_gfx()
        self.pop_autos()
        self.last_t = time.perf_counter()
        self.loop()

    def load_bg_img(self):
        if BG_FILE.exists():
            try:
                raw = Image.open(BG_FILE)
                # Crop to magenta border interior if present
                # magenta border: x=[122, 824], y=[19, 365]
                self.bg_img_raw = raw.crop((122, 19, 824, 365))
            except Exception as ex:
                print("Failed to load background image:", ex)
                self.bg_img_raw = None

    def update_bg_photo(self):
        if self.bg_img_raw is not None:
            cw = int(FL * self.zm)
            ch = int(FW * self.zm)
            if cw > 10 and ch > 10:
                resized = self.bg_img_raw.resize((cw, ch), Image.Resampling.BILINEAR)
                self.bg_img_tk = ImageTk.PhotoImage(resized)

    def ui(self):
        top = tk.Frame(self.root, bg="#282a36", padx=10, pady=8)
        top.pack(side=tk.TOP, fill=tk.X)

        self.pbtn = tk.Button(top, text="> Run All Autos", bg="#50fa7b", fg="#282a36", font=("Segoe UI", 10, "bold"),
                              padx=12, pady=2, command=self.toggle)
        self.pbtn.pack(side=tk.LEFT, padx=(5, 8))

        self.rbtn = tk.Button(top, text="Reset Sim", bg="#ff5555", fg="#ffffff", font=("Segoe UI", 10, "bold"),
                              padx=12, pady=2, command=self.reset)
        self.rbtn.pack(side=tk.LEFT, padx=5)

        tk.Label(top, text="Speed:", fg="#f8f8f2", bg="#282a36", font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=(15, 5))
        self.s_cb = ttk.Combobox(top, state="readonly", values=["0.5x", "1.0x", "2.0x", "4.0x"], width=5)
        self.s_cb.set("1.0x")
        self.s_cb.pack(side=tk.LEFT, padx=(0, 15))
        self.s_cb.bind("<<ComboboxSelected>>", lambda e: setattr(self, "spd", float(self.s_cb.get().replace("x", ""))))

        tk.Label(top, text="Quick Preset:", fg="#f8f8f2", bg="#282a36", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=(10, 5))
        self.pr_cb = ttk.Combobox(top, state="readonly",
                                  values=["All 4 Robots Charge Center", "2 Red vs 2 Blue", "Solo Red Top", "All Stay Still"], width=24)
        self.pr_cb.set("All 4 Robots Charge Center")
        self.pr_cb.pack(side=tk.LEFT, padx=(0, 15))
        self.pr_cb.bind("<<ComboboxSelected>>", self.on_preset)

        self.bld_btn = tk.Button(top, text="🛠️ Edit Obstacles", bg="#bd93f9", fg="#282a36", font=("Segoe UI", 10, "bold"),
                                 padx=10, pady=2, command=self.toggle_builder)
        self.bld_btn.pack(side=tk.LEFT, padx=(10, 5))

        self.stat_lbl = tk.Label(top, text="Ready", fg="#8be9fd", bg="#282a36", font=("Consolas", 10, "bold"))
        self.stat_lbl.pack(side=tk.RIGHT, padx=10)

        self.cnv = tk.Canvas(self.root, bg="#181920", highlightthickness=0)
        self.cnv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Bind canvas events for collision builder
        self.cnv.bind("<ButtonPress-1>", self.on_canvas_press)
        self.cnv.bind("<B1-Motion>", self.on_canvas_drag)
        self.cnv.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.cnv.bind("<Button-3>", self.on_canvas_right_click)

        side = tk.Frame(self.root, bg="#21222c", width=340, padx=12, pady=10)
        side.pack(side=tk.RIGHT, fill=tk.Y)
        side.pack_propagate(False)

        # Collision Builder Control Panel (Shown when editing)
        self.bld_panel = tk.LabelFrame(side, text=" 🛠️ COLLISION BUILDER ", fg="#bd93f9", bg="#282a36", font=("Segoe UI", 10, "bold"), padx=8, pady=6)
        
        bld_row1 = tk.Frame(self.bld_panel, bg="#282a36")
        bld_row1.pack(fill=tk.X, pady=2)
        tk.Label(bld_row1, text="Tool:", fg="#f8f8f2", bg="#282a36", font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        self.tool_cb = ttk.Combobox(bld_row1, state="readonly", values=[
            "Move/Resize Existing Box",
            "Solid Barrier",
            "Speed Bump",
            "Blue Hub (Center & Size)",
            "Red Hub (Center & Size)",
            "Neutral Zone (NZ)",
            "Blue Depot (BDP)",
            "Red Depot (RDP)"
        ], width=20)
        self.tool_cb.set("Move/Resize Existing Box")
        self.tool_cb.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(4, 0))
        self.tool_cb.bind("<<ComboboxSelected>>", self.on_tool_change)

        tk.Label(self.bld_panel, text="* Move/Resize: Drag boxes to move, or drag corner handles\n* Hub: Drag center dot to move, drag ring to resize\n* Right-Click box to delete", fg="#f1fa8c", bg="#282a36", font=("Segoe UI", 8), justify=tk.LEFT).pack(anchor="w", pady=(2, 3))

        bld_scale_row = tk.Frame(self.bld_panel, bg="#282a36")
        bld_scale_row.pack(fill=tk.X, pady=2)
        tk.Label(bld_scale_row, text="Field Size (m):", fg="#f8f8f2", bg="#282a36", font=("Segoe UI", 8)).pack(side=tk.LEFT)
        self.fl_entry = tk.Entry(bld_scale_row, width=6, bg="#181920", fg="#50fa7b", font=("Consolas", 8))
        self.fl_entry.insert(0, f"{FL:.2f}")
        self.fl_entry.pack(side=tk.LEFT, padx=2)
        tk.Label(bld_scale_row, text="x", fg="#f8f8f2", bg="#282a36").pack(side=tk.LEFT)
        self.fw_entry = tk.Entry(bld_scale_row, width=5, bg="#181920", fg="#50fa7b", font=("Consolas", 8))
        self.fw_entry.insert(0, f"{FW:.2f}")
        self.fw_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(bld_scale_row, text="Apply", bg="#bd93f9", fg="#282a36", font=("Segoe UI", 8, "bold"), command=self.apply_field_size).pack(side=tk.LEFT, padx=2)

        bld_row2 = tk.Frame(self.bld_panel, bg="#282a36")
        bld_row2.pack(fill=tk.X, pady=3)
        tk.Button(bld_row2, text="📋 Copy Numbers", bg="#f1fa8c", fg="#282a36", font=("Segoe UI", 8, "bold"), command=self.export_numbers).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        tk.Button(bld_row2, text="Save JSON", bg="#50fa7b", fg="#282a36", font=("Segoe UI", 8, "bold"), command=self.save_builder).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))

        bld_row3 = tk.Frame(self.bld_panel, bg="#282a36")
        bld_row3.pack(fill=tk.X, pady=2)
        tk.Button(bld_row3, text="Reset Default", bg="#ffb86c", fg="#282a36", font=("Segoe UI", 8, "bold"), command=self.reset_default_obstacles).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        tk.Button(bld_row3, text="Clear All", bg="#ff5555", fg="#ffffff", font=("Segoe UI", 8, "bold"), command=self.clear_obstacles).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))

        tk.Label(side, text="4-ROBOT CONTROLLER", fg="#50fa7b", bg="#21222c", font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=(8, 8))

        self.rcbs = []
        self.rhuds = []
        for i, (r, c) in enumerate(zip(self.robs, self.cfgs)):
            bx = tk.LabelFrame(side, text=f" {r.name} ", fg=r.col, bg="#282a36", font=("Segoe UI", 9, "bold"), padx=6, pady=4)
            bx.pack(fill=tk.X, pady=3)
            tr = tk.Frame(bx, bg="#282a36")
            tr.pack(fill=tk.X)
            tk.Label(tr, text="Auto:", fg="#f8f8f2", bg="#282a36", font=("Segoe UI", 8)).pack(side=tk.LEFT)
            cb = ttk.Combobox(tr, state="readonly", width=22)
            cb.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(4, 0))
            cb.bind("<<ComboboxSelected>>", lambda e, idx=i: self.on_sel(idx))
            self.rcbs.append(cb)
            lbl = tk.Label(bx, text="Hopper: 00/50 | Speed: 0.00 m/s\nCmd: Idle", fg="#f8f8f2", bg="#282a36", font=("Consolas", 8), justify=tk.LEFT, anchor="w")
            lbl.pack(fill=tk.X, pady=(2, 0))
            self.rhuds.append(lbl)

        mbx = tk.LabelFrame(side, text=" FIELD & SCORING ", fg="#ff79c6", bg="#282a36", font=("Segoe UI", 9, "bold"), padx=8, pady=6)
        mbx.pack(fill=tk.X, pady=(12, 6))
        self.tb_lbl = tk.Label(mbx, text="Field FUEL Remaining: 408", fg="#f1fa8c", bg="#282a36", font=("Segoe UI", 9, "bold"))
        mbx.pack(fill=tk.X, pady=(12, 6))
        self.tb_lbl.pack(anchor="w")
        self.sc_lbl = tk.Label(mbx, text="Scored Balls: 0", fg="#ffb86c", bg="#282a36", font=("Segoe UI", 9, "bold"))
        self.sc_lbl.pack(anchor="w")

    def on_tool_change(self, e=None):
        val = self.tool_cb.get()
        if "Move/Resize" in val: self.builder_tool = "select"
        elif "Barrier" in val: self.builder_tool = "barrier"
        elif "Bump" in val: self.builder_tool = "bump"
        elif "NZ" in val: self.builder_tool = "nz"
        elif "BDP" in val: self.builder_tool = "bdp"
        elif "RDP" in val: self.builder_tool = "rdp"
        elif "Blue Hub" in val: self.builder_tool = "bhub"
        elif "Red Hub" in val: self.builder_tool = "rhub"

    def apply_field_size(self):
        global FL, FW
        try:
            n_fl = float(self.fl_entry.get())
            n_fw = float(self.fw_entry.get())
            if n_fl > 2.0 and n_fw > 2.0:
                FL = n_fl
                FW = n_fw
                CALIB["FL"] = round(FL, 3)
                CALIB["FW"] = round(FW, 3)
                save_calib(CALIB)
                self.init_gfx()
                self.stat_lbl.config(text=f"Updated Field Size to {FL:.2f}m x {FW:.2f}m", fg="#50fa7b")
        except Exception as ex:
            self.stat_lbl.config(text=f"Invalid Field Size: {ex}", fg="#ff5555")

    def export_numbers(self):
        calib_data = {
            "FL": round(FL, 3),
            "FW": round(FW, 3),
            "BHUB": [round(BHUB[0], 3), round(BHUB[1], 3)],
            "RHUB": [round(RHUB[0], 3), round(RHUB[1], 3)],
            "BHUB_RAD": round(BHUB_RAD, 3),
            "RHUB_RAD": round(RHUB_RAD, 3),
            "NZ": [round(x, 3) for x in CALIB["NZ"]],
            "BDP": [round(x, 3) for x in CALIB["BDP"]],
            "RDP": [round(x, 3) for x in CALIB["RDP"]],
            "obstacles": self.fld.obstacles
        }
        json_str = json.dumps(calib_data, indent=2)

        # Copy to system clipboard
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(json_str)
        except Exception:
            pass

        # Pop up dialog so user can select/copy easily
        dlg = tk.Toplevel(self.root)
        dlg.title("📋 Field Numbers & Obstacles (Copied to Clipboard!)")
        dlg.geometry("640x520")
        dlg.configure(bg="#21222c")

        tk.Label(dlg, text="Field Numbers & Coordinates (Auto-copied to clipboard!):", fg="#50fa7b", bg="#21222c", font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=10, pady=(10, 5))
        txt = tk.Text(dlg, bg="#181920", fg="#f8f8f2", font=("Consolas", 9), insertbackground="#ffffff", relief=tk.FLAT)
        txt.insert("1.0", json_str)
        txt.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        btn_bar = tk.Frame(dlg, bg="#21222c")
        btn_bar.pack(fill=tk.X, padx=10, pady=8)
        tk.Button(btn_bar, text="Copy Again", bg="#bd93f9", fg="#282a36", font=("Segoe UI", 9, "bold"), command=lambda: (self.root.clipboard_clear(), self.root.clipboard_append(txt.get("1.0", tk.END)))).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_bar, text="Close", bg="#6272a4", fg="#ffffff", font=("Segoe UI", 9, "bold"), command=dlg.destroy).pack(side=tk.RIGHT, padx=5)

        self.stat_lbl.config(text="Numbers copied to clipboard!", fg="#50fa7b")

    def toggle_builder(self):
        self.builder_mode = not self.builder_mode
        if self.builder_mode:
            self.bld_btn.config(text="✔ Done Editing", bg="#50fa7b")
            self.bld_panel.pack(fill=tk.X, pady=(0, 10), before=self.rcbs[0].master.master)
            self.stat_lbl.config(text="Collision Builder Active: Click & Drag to Draw", fg="#bd93f9")
            # Pause sim while editing
            for e in self.engs:
                e.stop()
            self.pbtn.config(text="> Resume", bg="#50fa7b")
        else:
            self.bld_btn.config(text="🛠️ Edit Obstacles", bg="#bd93f9")
            self.bld_panel.pack_forget()
            self.stat_lbl.config(text="Editing Done - Obstacles Active", fg="#50fa7b")
        self.init_gfx()

    def save_builder(self):
        save_obstacles(self.fld.obstacles)
        save_calib(CALIB)
        self.stat_lbl.config(text="Saved Obstacles & Calibration to JSON!", fg="#50fa7b")

    def reset_default_obstacles(self):
        global FL, FW, BHUB, RHUB, BHUB_RAD, RHUB_RAD, NZ_X0, NZ_Y0, NZ_X1, NZ_Y1, BDP_X0, BDP_Y0, BDP_X1, BDP_Y1, RDP_X0, RDP_Y0, RDP_X1, RDP_Y1
        self.fld.obstacles = [dict(o) for o in DEFAULT_OBSTACLES]
        save_obstacles(self.fld.obstacles)
        for k, v in DEFAULT_CALIB.items():
            CALIB[k] = v
        save_calib(CALIB)
        FL = float(CALIB["FL"])
        FW = float(CALIB["FW"])
        BHUB = tuple(CALIB["BHUB"])
        RHUB = tuple(CALIB["RHUB"])
        BHUB_RAD = float(CALIB.get("BHUB_RAD", 0.60))
        RHUB_RAD = float(CALIB.get("RHUB_RAD", 0.60))
        NZ_X0, NZ_Y0, NZ_X1, NZ_Y1 = CALIB["NZ"]
        BDP_X0, BDP_Y0, BDP_X1, BDP_Y1 = CALIB["BDP"]
        RDP_X0, RDP_Y0, RDP_X1, RDP_Y1 = CALIB["RDP"]
        self.fl_entry.delete(0, tk.END)
        self.fl_entry.insert(0, f"{FL:.2f}")
        self.fw_entry.delete(0, tk.END)
        self.fw_entry.insert(0, f"{FW:.2f}")
        for r in self.robs:
            r._obstacles = self.fld.obstacles
        self.fld.reset_balls()
        self.init_gfx()
        self.stat_lbl.config(text="Reset to Default Boxes & Layout", fg="#ffb86c")

    def clear_obstacles(self):
        self.fld.obstacles = []
        save_obstacles(self.fld.obstacles)
        for r in self.robs:
            r._obstacles = self.fld.obstacles
        self.init_gfx()
        self.stat_lbl.config(text="All Obstacles Cleared", fg="#ff5555")

    def s2w(self, sx, sy):
        wx = (sx - self.ox) / self.zm
        wy = FW - (sy - self.oy) / self.zm
        return wx, wy

    def get_hit_target(self, wx, wy):
        tol = 0.25  # hit radius in meters
        # 1. Check Hub handles (Center vs Radius Ring)
        for hub_key, hub_coord, hub_rad, hub_name in [("bhub", BHUB, BHUB_RAD, "Blue Hub"), ("rhub", RHUB, RHUB_RAD, "Red Hub")]:
            dist = math.hypot(wx - hub_coord[0], wy - hub_coord[1])
            if dist < 0.25:
                return {"target": hub_key, "handle": "move", "name": f"{hub_name} Center"}
            elif abs(dist - hub_rad) < 0.25:
                return {"target": hub_key, "handle": "radius", "name": f"{hub_name} Ring"}

        # 2. Check special zones: NZ, BDP, RDP handles
        zones = [
            ("nz", CALIB["NZ"], "Neutral Zone"),
            ("bdp", CALIB["BDP"], "Blue Depot"),
            ("rdp", CALIB["RDP"], "Red Depot")
        ]
        for z_key, coords, z_name in zones:
            x0, y0, x1, y1 = coords
            # Corner handles
            corners = {
                "nw": (x0, y1),
                "ne": (x1, y1),
                "se": (x1, y0),
                "sw": (x0, y0)
            }
            for c_name, (cx, cy) in corners.items():
                if math.hypot(wx - cx, wy - cy) < tol:
                    return {"target": z_key, "handle": c_name, "name": f"{z_name} {c_name.upper()} handle"}
            # Inside zone body
            if min(x0, x1) <= wx <= max(x0, x1) and min(y0, y1) <= wy <= max(y0, y1):
                return {"target": z_key, "handle": "move", "name": z_name}

        # 3. Check obstacles (Barriers and Bumps)
        for idx, obs in enumerate(self.fld.obstacles):
            x0, x1 = min(obs["x0"], obs["x1"]), max(obs["x0"], obs["x1"])
            y0, y1 = min(obs["y0"], obs["y1"]), max(obs["y0"], obs["y1"])
            corners = {
                "nw": (x0, y1),
                "ne": (x1, y1),
                "se": (x1, y0),
                "sw": (x0, y0)
            }
            for c_name, (cx, cy) in corners.items():
                if math.hypot(wx - cx, wy - cy) < tol:
                    return {"target": idx, "handle": c_name, "name": f"{obs.get('name', 'Box')} {c_name.upper()}"}
            if x0 <= wx <= x1 and y0 <= wy <= y1:
                return {"target": idx, "handle": "move", "name": obs.get("name", "Box")}

        return None

    def on_canvas_press(self, event):
        global BHUB, RHUB
        if not self.builder_mode:
            return
        wx, wy = self.s2w(event.x, event.y)
        self.drag_start = (wx, wy)
        self.drag_start_pix = (event.x, event.y)
        if self.preview_rect_id:
            self.cnv.delete(self.preview_rect_id)
            self.preview_rect_id = None

        # When in select mode OR clicking an existing element:
        hit = self.get_hit_target(wx, wy)
        if self.builder_tool == "select" or (hit and self.builder_tool in ("select", "bhub", "rhub")):
            if hit:
                self.drag_target = hit["target"]
                self.drag_handle = hit["handle"]
                if self.drag_target == "bhub":
                    self.drag_init_geom = (BHUB[0], BHUB[1], BHUB_RAD)
                elif self.drag_target == "rhub":
                    self.drag_init_geom = (RHUB[0], RHUB[1], RHUB_RAD)
                elif self.drag_target in ("nz", "bdp", "rdp"):
                    self.drag_init_geom = list(CALIB[self.drag_target.upper()])
                else:
                    obs = self.fld.obstacles[self.drag_target]
                    self.drag_init_geom = (obs["x0"], obs["y0"], obs["x1"], obs["y1"])
                self.stat_lbl.config(text=f"Dragging: {hit['name']}", fg="#50fa7b")
                return

        # Direct Hub placement if tool specifically chosen:
        if self.builder_tool in ("bhub", "rhub"):
            if self.builder_tool == "bhub":
                BHUB = (round(wx, 3), round(wy, 3))
                CALIB["BHUB"] = list(BHUB)
                save_calib(CALIB)
                self.stat_lbl.config(text=f"Set Blue Hub Center to ({BHUB[0]:.2f}, {BHUB[1]:.2f})", fg="#8be9fd")
            else:
                RHUB = (round(wx, 3), round(wy, 3))
                CALIB["RHUB"] = list(RHUB)
                save_calib(CALIB)
                self.stat_lbl.config(text=f"Set Red Hub Center to ({RHUB[0]:.2f}, {RHUB[1]:.2f})", fg="#ff5555")
            self.init_gfx()
            self.drag_start = None
            return

        self.drag_target = None
        self.drag_handle = None

    def on_canvas_drag(self, event):
        if not self.builder_mode or not self.drag_start:
            return
        wx, wy = self.s2w(event.x, event.y)
        dwx = wx - self.drag_start[0]
        dwy = wy - self.drag_start[1]

        # Case A: Interacting with existing target (Move or Resize)
        if self.drag_target is not None:
            global BHUB, RHUB, BHUB_RAD, RHUB_RAD, NZ_X0, NZ_Y0, NZ_X1, NZ_Y1, BDP_X0, BDP_Y0, BDP_X1, BDP_Y1, RDP_X0, RDP_Y0, RDP_X1, RDP_Y1
            # 1. Blue / Red Hub
            if self.drag_target in ("bhub", "rhub"):
                orig_x, orig_y, orig_r = self.drag_init_geom
                if self.drag_handle == "move":
                    nx = round(max(0.0, min(FL, orig_x + dwx)), 3)
                    ny = round(max(0.0, min(FW, orig_y + dwy)), 3)
                    if self.drag_target == "bhub":
                        BHUB = (nx, ny)
                        CALIB["BHUB"] = [nx, ny]
                    else:
                        RHUB = (nx, ny)
                        CALIB["RHUB"] = [nx, ny]
                elif self.drag_handle == "radius":
                    cur_r = round(max(0.20, math.hypot(wx - orig_x, wy - orig_y)), 3)
                    if self.drag_target == "bhub":
                        BHUB_RAD = cur_r
                        CALIB["BHUB_RAD"] = cur_r
                    else:
                        RHUB_RAD = cur_r
                        CALIB["RHUB_RAD"] = cur_r
                save_calib(CALIB)
                self.init_gfx()
                return

            # 2. Zone or Obstacle Box
            ox0, oy0, ox1, oy1 = self.drag_init_geom
            x0, y0, x1, y1 = min(ox0, ox1), min(oy0, oy1), max(ox0, ox1), max(oy0, oy1)

            if self.drag_handle == "move":
                x0 = max(0.0, min(FL, x0 + dwx))
                x1 = max(0.0, min(FL, x1 + dwx))
                y0 = max(0.0, min(FW, y0 + dwy))
                y1 = max(0.0, min(FW, y1 + dwy))
            elif self.drag_handle == "nw":
                x0 = max(0.0, min(x1 - 0.1, x0 + dwx))
                y1 = max(y0 + 0.1, min(FW, y1 + dwy))
            elif self.drag_handle == "ne":
                x1 = max(x0 + 0.1, min(FL, x1 + dwx))
                y1 = max(y0 + 0.1, min(FW, y1 + dwy))
            elif self.drag_handle == "se":
                x1 = max(x0 + 0.1, min(FL, x1 + dwx))
                y0 = max(0.0, min(y1 - 0.1, y0 + dwy))
            elif self.drag_handle == "sw":
                x0 = max(0.0, min(x1 - 0.1, x0 + dwx))
                y0 = max(0.0, min(y1 - 0.1, y0 + dwy))

            new_box = [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)]
            if self.drag_target == "nz":
                CALIB["NZ"] = new_box
                NZ_X0, NZ_Y0, NZ_X1, NZ_Y1 = new_box
                save_calib(CALIB)
            elif self.drag_target == "bdp":
                CALIB["BDP"] = new_box
                BDP_X0, BDP_Y0, BDP_X1, BDP_Y1 = new_box
                save_calib(CALIB)
            elif self.drag_target == "rdp":
                CALIB["RDP"] = new_box
                RDP_X0, RDP_Y0, RDP_X1, RDP_Y1 = new_box
                save_calib(CALIB)
            else:
                obs = self.fld.obstacles[self.drag_target]
                obs["x0"], obs["y0"], obs["x1"], obs["y1"] = round(x0, 2), round(y0, 2), round(x1, 2), round(y1, 2)
                save_obstacles(self.fld.obstacles)
                for r in self.robs:
                    r._obstacles = self.fld.obstacles
            self.init_gfx()
            return

        # Case B: Creating new box
        sx0, sy0 = self.drag_start_pix
        sx1, sy1 = event.x, event.y
        col = "#bd93f9" if self.builder_tool == "barrier" else ("#ffb86c" if self.builder_tool == "bump" else "#50fa7b")
        if not self.preview_rect_id:
            self.preview_rect_id = self.cnv.create_rectangle(sx0, sy0, sx1, sy1, outline=col, dash=(3, 3), width=2, tags="builder_preview")
        else:
            self.cnv.coords(self.preview_rect_id, sx0, sy0, sx1, sy1)

    def on_canvas_release(self, event):
        if not self.builder_mode or not self.drag_start:
            return
        if self.preview_rect_id:
            self.cnv.delete(self.preview_rect_id)
            self.preview_rect_id = None

        # Finished moving/resizing existing target
        if self.drag_target is not None:
            if self.drag_target in ("nz", "bdp", "rdp"):
                self.fld.reset_balls()
            self.drag_target = None
            self.drag_handle = None
            self.drag_start = None
            self.stat_lbl.config(text="Box Updated & Saved", fg="#50fa7b")
            self.init_gfx()
            return

        # Draw brand new box
        sx0, sy0 = self.drag_start_pix
        sx1, sy1 = event.x, event.y
        self.drag_start = None

        if abs(sx1 - sx0) < 6 or abs(sy1 - sy0) < 6:
            return

        w0x, w0y = self.s2w(min(sx0, sx1), max(sy0, sy1))
        w1x, w1y = self.s2w(max(sx0, sx1), min(sy0, sy1))
        x0, x1 = max(0.0, min(FL, min(w0x, w1x))), max(0.0, min(FL, max(w0x, w1x)))
        y0, y1 = max(0.0, min(FW, min(w0y, w1y))), max(0.0, min(FW, max(w0y, w1y)))

        global NZ_X0, NZ_Y0, NZ_X1, NZ_Y1, BDP_X0, BDP_Y0, BDP_X1, BDP_Y1, RDP_X0, RDP_Y0, RDP_X1, RDP_Y1
        if self.builder_tool == "nz":
            CALIB["NZ"] = [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)]
            NZ_X0, NZ_Y0, NZ_X1, NZ_Y1 = CALIB["NZ"]
            save_calib(CALIB)
            self.fld.reset_balls()
            self.init_gfx()
            self.stat_lbl.config(text=f"Updated Neutral Zone: [{x0:.2f}, {y0:.2f}] to [{x1:.2f}, {y1:.2f}]", fg="#50fa7b")
            return
        elif self.builder_tool == "bdp":
            CALIB["BDP"] = [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)]
            BDP_X0, BDP_Y0, BDP_X1, BDP_Y1 = CALIB["BDP"]
            save_calib(CALIB)
            self.fld.reset_balls()
            self.init_gfx()
            self.stat_lbl.config(text=f"Updated Blue Depot: [{x0:.2f}, {y0:.2f}] to [{x1:.2f}, {y1:.2f}]", fg="#8be9fd")
            return
        elif self.builder_tool == "rdp":
            CALIB["RDP"] = [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)]
            RDP_X0, RDP_Y0, RDP_X1, RDP_Y1 = CALIB["RDP"]
            save_calib(CALIB)
            self.fld.reset_balls()
            self.init_gfx()
            self.stat_lbl.config(text=f"Updated Red Depot: [{x0:.2f}, {y0:.2f}] to [{x1:.2f}, {y1:.2f}]", fg="#ff5555")
            return

        name = f"{'Barrier' if self.builder_tool == 'barrier' else 'Bump'} {len(self.fld.obstacles)+1}"
        new_obs = {
            "name": name,
            "type": "bump" if self.builder_tool == "bump" else "barrier",
            "x0": round(x0, 2),
            "y0": round(y0, 2),
            "x1": round(x1, 2),
            "y1": round(y1, 2)
        }
        self.fld.obstacles.append(new_obs)
        save_obstacles(self.fld.obstacles)
        for r in self.robs:
            r._obstacles = self.fld.obstacles
        self.init_gfx()
        self.stat_lbl.config(text=f"Added {name} [{x0:.2f}, {y0:.2f}] to [{x1:.2f}, {y1:.2f}]", fg="#50fa7b")

    def on_canvas_right_click(self, event):
        if not self.builder_mode:
            return
        wx, wy = self.s2w(event.x, event.y)
        # Check if clicked inside any obstacle to delete it
        rem_idx = None
        for idx, obs in enumerate(self.fld.obstacles):
            x0, x1 = min(obs["x0"], obs["x1"]), max(obs["x0"], obs["x1"])
            y0, y1 = min(obs["y0"], obs["y1"]), max(obs["y0"], obs["y1"])
            if x0 <= wx <= x1 and y0 <= wy <= y1:
                rem_idx = idx
                break
        if rem_idx is not None:
            deleted = self.fld.obstacles.pop(rem_idx)
            save_obstacles(self.fld.obstacles)
            for r in self.robs:
                r._obstacles = self.fld.obstacles
            self.init_gfx()
            self.stat_lbl.config(text=f"Deleted {deleted.get('name', 'Obstacle')}", fg="#ff5555")

    def pop_autos(self):
        afs = [f.name for f in adir.glob("*.auto")]
        opts = ["Stay Still (Idle)"] + sorted(afs)
        for i, (cb, c) in enumerate(zip(self.rcbs, self.cfgs)):
            cb["values"] = opts
            d = c["def"] if c["def"] in afs else "Stay Still (Idle)"
            cb.set(d)
            self.engs[i].load_auto(d, mx=c["mx"], my=c["my"])

    def on_sel(self, idx):
        s = self.rcbs[idx].get()
        c = self.cfgs[idx]
        self.engs[idx].load_auto(s, mx=c["mx"], my=c["my"])

    def on_preset(self, e):
        p = self.pr_cb.get()
        afs = [f.name for f in adir.glob("*.auto")]
        if p == "All 4 Robots Charge Center":
            self.rcbs[0].set("L-Double-Tap-Trench-Depot.auto" if "L-Double-Tap-Trench-Depot.auto" in afs else afs[0])
            self.rcbs[1].set("R-Double-Tap-Trench.auto" if "R-Double-Tap-Trench.auto" in afs else afs[0])
            self.rcbs[2].set("L-Double-Tap-Trench-Depot.auto" if "L-Double-Tap-Trench-Depot.auto" in afs else afs[0])
            self.rcbs[3].set("R-Double-Tap-Trench.auto" if "R-Double-Tap-Trench.auto" in afs else afs[0])
        elif p == "2 Red vs 2 Blue":
            self.rcbs[0].set("L-Triple-Tap-Trench.auto" if "L-Triple-Tap-Trench.auto" in afs else afs[0])
            self.rcbs[1].set("R-Triple-Tap-Trench.auto" if "R-Triple-Tap-Trench.auto" in afs else afs[0])
            self.rcbs[2].set("L-Triple-Tap-Trench.auto" if "L-Triple-Tap-Trench.auto" in afs else afs[0])
            self.rcbs[3].set("R-Triple-Tap-Trench.auto" if "R-Triple-Tap-Trench.auto" in afs else afs[0])
        elif p == "Solo Red Top":
            self.rcbs[0].set("L-Double-Tap-Trench-Depot.auto" if "L-Double-Tap-Trench-Depot.auto" in afs else afs[0])
            self.rcbs[1].set("Stay Still (Idle)")
            self.rcbs[2].set("Stay Still (Idle)")
            self.rcbs[3].set("Stay Still (Idle)")
        elif p == "All Stay Still":
            for cb in self.rcbs:
                cb.set("Stay Still (Idle)")
        for i, cb in enumerate(self.rcbs):
            self.on_sel(i)
        self.reset()

    def toggle(self):
        if any(e.active for e in self.engs):
            for e in self.engs:
                e.stop()
            self.pbtn.config(text="> Resume", bg="#50fa7b")
        else:
            for e in self.engs:
                e.start()
            self.pbtn.config(text="|| Pause", bg="#ffb86c")

    def reset(self):
        for e in self.engs:
            e.stop()
            e.reset_robot_pose()
        self.fld.reset_balls()
        self.init_gfx()
        for i, cb in enumerate(self.rcbs):
            self.on_sel(i)
        self.pbtn.config(text="> Run All Autos", bg="#50fa7b")
        self.stat_lbl.config(text="Simulator Reset")

    def w2s(self, x, y):
        return (self.ox + x * self.zm, self.oy + (FW - y) * self.zm)

    def init_gfx(self):
        self.cnv.delete("all")
        self.b_items.clear()
        tl = self.w2s(0, FW)
        br = self.w2s(FL, 0)

        # Background Field Image
        self.update_bg_photo()
        if self.bg_img_tk:
            self.cnv.create_image(tl[0], tl[1], anchor=tk.NW, image=self.bg_img_tk, tags="static")
        else:
            self.cnv.create_rectangle(tl[0], tl[1], br[0], br[1], outline="#44475a", fill="#282a36", width=2, tags="static")

        ct = self.w2s(FL / 2.0, FW)
        cb = self.w2s(FL / 2.0, 0)
        self.cnv.create_line(ct[0], ct[1], cb[0], cb[1], fill="#6272a4", dash=(4, 4), width=2, tags="static")

        nz_tl = self.w2s(NZ_X0, NZ_Y1)
        nz_br = self.w2s(NZ_X1, NZ_Y0)
        self.cnv.create_rectangle(nz_tl[0], nz_tl[1], nz_br[0], nz_br[1], outline="#6272a4", fill="", dash=(2, 2), width=1, tags="static")

        b_tl = self.w2s(BDP_X0, BDP_Y1)
        b_br = self.w2s(BDP_X1, BDP_Y0)
        self.cnv.create_rectangle(b_tl[0], b_tl[1], b_br[0], b_br[1], outline="#8be9fd", fill="", width=2, tags="static")

        r_tl = self.w2s(RDP_X0, RDP_Y1)
        r_br = self.w2s(RDP_X1, RDP_Y0)
        self.cnv.create_rectangle(r_tl[0], r_tl[1], r_br[0], r_br[1], outline="#ff5555", fill="", width=2, tags="static")

        hr = self.w2s(RHUB[0], RHUB[1])
        hb = self.w2s(BHUB[0], BHUB[1])
        rad_r = RHUB_RAD * self.zm
        rad_b = BHUB_RAD * self.zm
        self.cnv.create_oval(hr[0] - rad_r, hr[1] - rad_r, hr[0] + rad_r, hr[1] + rad_r, outline="#ff5555", fill="", width=3, tags="static")
        self.cnv.create_oval(hb[0] - rad_b, hb[1] - rad_b, hb[0] + rad_b, hb[1] + rad_b, outline="#8be9fd", fill="", width=3, tags="static")
        if self.builder_mode:
            # Hub center crosshairs and radius labels
            self.cnv.create_oval(hr[0] - 4, hr[1] - 4, hr[0] + 4, hr[1] + 4, fill="#ff5555", outline="#ffffff", tags="obstacle")
            self.cnv.create_oval(hb[0] - 4, hb[1] - 4, hb[0] + 4, hb[1] + 4, fill="#8be9fd", outline="#ffffff", tags="obstacle")
            self.cnv.create_text(hr[0], hr[1] - rad_r - 12, text=f"Red Hub (r={RHUB_RAD:.2f}m)", fill="#ff5555", font=("Segoe UI", 8, "bold"), tags="obstacle")
            self.cnv.create_text(hb[0], hb[1] - rad_b - 12, text=f"Blue Hub (r={BHUB_RAD:.2f}m)", fill="#8be9fd", font=("Segoe UI", 8, "bold"), tags="obstacle")

        # Render Obstacles (Hub Barriers and Speed Bumps)
        for idx, obs in enumerate(self.fld.obstacles):
            x0, y0 = min(obs["x0"], obs["x1"]), min(obs["y0"], obs["y1"])
            x1, y1 = max(obs["x0"], obs["x1"]), max(obs["y0"], obs["y1"])
            otl = self.w2s(x0, y1)
            obr = self.w2s(x1, y0)
            otype = obs.get("type", "barrier")
            if otype == "barrier":
                # Solid red/blue barrier boundary
                col = "#bd93f9" if self.builder_mode else "#ff5555"
                self.cnv.create_rectangle(otl[0], otl[1], obr[0], obr[1], outline=col, width=3, dash=(4, 2) if self.builder_mode else (), tags="obstacle")
                if self.builder_mode:
                    self.cnv.create_text((otl[0]+obr[0])/2, (otl[1]+obr[1])/2, text=obs.get("name", "Barrier"), fill=col, font=("Segoe UI", 9, "bold"), tags="obstacle")
            else:
                # Speed Bump zone
                bcol = "#ffb86c"
                self.cnv.create_rectangle(otl[0], otl[1], obr[0], obr[1], outline=bcol, width=2, dash=(6, 3), tags="obstacle")
                # Hatch lines across bump
                mid_y = (otl[1] + obr[1]) / 2
                self.cnv.create_line(otl[0], mid_y, obr[0], mid_y, fill=bcol, dash=(3, 3), width=1, tags="obstacle")
                if self.builder_mode:
                    self.cnv.create_text((otl[0]+obr[0])/2, (otl[1]+obr[1])/2, text=obs.get("name", "Bump"), fill=bcol, font=("Segoe UI", 8, "bold"), tags="obstacle")

            if self.builder_mode:
                # Draw draggable corner handles
                for cx, cy in [(x0, y1), (x1, y1), (x1, y0), (x0, y0)]:
                    hc = self.w2s(cx, cy)
                    self.cnv.create_rectangle(hc[0] - 4, hc[1] - 4, hc[0] + 4, hc[1] + 4, fill="#50fa7b", outline="#181920", tags="obstacle")

        if self.builder_mode:
            # Draggable handles for NZ, BDP, RDP
            for z_coords, hcol in [(CALIB["NZ"], "#6272a4"), (CALIB["BDP"], "#8be9fd"), (CALIB["RDP"], "#ff5555")]:
                zx0, zy0, zx1, zy1 = z_coords
                for cx, cy in [(zx0, zy1), (zx1, zy1), (zx1, zy0), (zx0, zy0)]:
                    hc = self.w2s(cx, cy)
                    self.cnv.create_rectangle(hc[0] - 4, hc[1] - 4, hc[0] + 4, hc[1] + 4, fill=hcol, outline="#ffffff", tags="obstacle")

        for b in self.fld.balls:
            bx, by = self.w2s(b.x, b.y)
            rad_b = b.r * self.zm
            self.b_items[b.id] = self.cnv.create_oval(bx - rad_b, by - rad_b, bx + rad_b, by + rad_b, fill="#f1fa8c", outline="#ffb86c", width=1, tags="ball")
        self.init_done = True

    def loop(self):
        now = time.perf_counter()
        dt = min(0.08, (now - self.last_t) * self.spd)
        self.last_t = now

        for e in self.engs:
            e.update(dt)
        col_robs(self.robs)
        for r in self.robs:
            r.update_intake(self.fld, dt)
        self.fld.step(dt)

        self.draw()
        self.hud()

        calc = time.perf_counter() - now
        self.root.after(max(2, int((0.02 - calc) * 1000)), self.loop)

    def draw(self):
        if not self.init_done:
            self.init_gfx()
        act_ids = set()
        for b in self.fld.balls:
            act_ids.add(b.id)
            bx, by = self.w2s(b.x, b.y)
            rad_b = b.r * self.zm
            if b.id in self.b_items:
                self.cnv.coords(self.b_items[b.id], bx - rad_b, by - rad_b, bx + rad_b, by + rad_b)
            else:
                self.b_items[b.id] = self.cnv.create_oval(bx - rad_b, by - rad_b, bx + rad_b, by + rad_b, fill="#f1fa8c", outline="#ffb86c", width=1, tags="ball")
        for bid, itm in list(self.b_items.items()):
            if bid not in act_ids:
                self.cnv.coords(itm, -200, -200, -200, -200)

        self.cnv.delete("dynamic")
        for p in self.fld.projs:
            px, py = self.w2s(p.cx, p.cy)
            psz = 0.075 * self.zm * (1.0 + p.ah * 0.4)
            self.cnv.create_oval(px - psz, py - psz, px + psz, py + psz, fill="#ff79c6", outline="#ffffff", width=2, tags="dynamic")

        for r in self.robs:
            pts = []
            for pt in [self.w2s(x, y) for x, y in r.corners()]:
                pts.extend([pt[0], pt[1]])
            col = r.col if not r.shooting else "#f1fa8c"
            self.cnv.create_polygon(pts, outline="#f8f8f2", fill=col, width=2, tags="dynamic")

            c_scr = self.w2s(r.x, r.y)
            self.cnv.create_text(c_scr[0], c_scr[1], text=r.name.split()[0], fill="#181920", font=("Segoe UI", 8, "bold"), tags="dynamic")

            ipts = []
            for pt in [self.w2s(x, y) for x, y in r.intake_poly()]:
                ipts.extend([pt[0], pt[1]])
            icol = "#50fa7b" if r.intake else "#2d7f4a"
            self.cnv.create_polygon(ipts, outline="#50fa7b", fill=icol, width=2, tags="dynamic")

            p1, p2 = r.intake_front()
            sp1, sp2 = self.w2s(p1[0], p1[1]), self.w2s(p2[0], p2[1])
            self.cnv.create_line(sp1[0], sp1[1], sp2[0], sp2[1], fill="#ffffff", width=3, tags="dynamic")

            sp = r.shooter_pos()
            ss = self.w2s(sp[0], sp[1])
            sr = 0.12 * self.zm
            sfill = "#ff79c6" if r.shooting else "#ffb86c"
            self.cnv.create_oval(ss[0] - sr, ss[1] - sr, ss[0] + sr, ss[1] + sr, fill=sfill, outline="#ffffff", width=2, tags="dynamic")

            if r.shooting:
                thub = self.fld.tgt_hub(r.al)
                ths = self.w2s(thub[0], thub[1])
                self.cnv.create_line(ss[0], ss[1], ths[0], ths[1], fill="#ff79c6", dash=(3, 3), width=2, tags="dynamic")

            hx = r.x + 0.55 * math.cos(r.th)
            hy = r.y + 0.55 * math.sin(r.th)
            hs = self.w2s(hx, hy)
            self.cnv.create_line(c_scr[0], c_scr[1], hs[0], hs[1], fill="#50fa7b", arrow=tk.LAST, width=2, tags="dynamic")

    def hud(self):
        for i, (r, e, lbl) in enumerate(zip(self.robs, self.engs, self.rhuds)):
            c = e.cur_cmd
            if len(c) > 22:
                c = c[:20] + ".."
            sp = math.hypot(r.vx, r.vy)
            lbl.config(text=f"Hopper: {r.hp:02d}/50 | Speed: {sp:.2f} m/s\nCmd: {c}")

        self.tb_lbl.config(text=f"Field FUEL Remaining: {len(self.fld.balls)}")
        self.sc_lbl.config(text=f"Scored Balls in Hubs: {self.fld.scored}")

        act = [r.name for r, e in zip(self.robs, self.engs) if e.active]
        self.stat_lbl.config(text=f"Running: {len(act)} Active" if act else "Idle / Ready", fg="#50fa7b" if act else "#8be9fd")

if __name__ == "__main__":
    rt = tk.Tk()
    app = Sim(rt)
    rt.mainloop()
