import tkinter as tk
from tkinter import ttk
import math, time, json, bisect, random
from pathlib import Path

rdir = Path(__file__).resolve().parent / "src" / "main" / "deploy" / "pathplanner"
if not rdir.exists():
    rdir = Path("src/main/deploy/pathplanner")
sf = rdir / "settings.json"
adir = rdir / "autos"
pdir = rdir / "paths"

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

FL = 16.532
FW = 8.001
BHUB = (3.465 + 23.5 * 0.0254, 4.0)
RHUB = (13.067 - 23.5 * 0.0254, 4.0)

BD = 5.91 * 0.0254
BR = BD / 2.0
MAX_HP = 50

NZ_CX, NZ_CY = FL / 2.0, FW / 2.0
NZ_X0, NZ_X1 = NZ_CX - 1.10, NZ_CX + 1.10
NZ_Y0, NZ_Y1 = NZ_CY - 2.90, NZ_CY + 2.90

DP_DX = 27.0 * 0.0254
DP_WY = 42.0 * 0.0254
BDP_X0, BDP_X1 = 0.0, DP_DX
BDP_Y0, BDP_Y1 = 5.50, 5.50 + DP_WY
RDP_X0, RDP_X1 = FL - DP_DX, FL
RDP_Y0, RDP_Y1 = 1.434, 1.434 + DP_WY

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

        self.ui()
        self.init_gfx()
        self.pop_autos()
        self.last_t = time.perf_counter()
        self.loop()

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

        self.stat_lbl = tk.Label(top, text="Ready", fg="#8be9fd", bg="#282a36", font=("Consolas", 10, "bold"))
        self.stat_lbl.pack(side=tk.RIGHT, padx=10)

        self.cnv = tk.Canvas(self.root, bg="#181920", highlightthickness=0)
        self.cnv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        side = tk.Frame(self.root, bg="#21222c", width=340, padx=12, pady=10)
        side.pack(side=tk.RIGHT, fill=tk.Y)
        side.pack_propagate(False)

        tk.Label(side, text="4-ROBOT CONTROLLER", fg="#50fa7b", bg="#21222c", font=("Segoe UI", 12, "bold")).pack(anchor="w", pady=(0, 8))

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
        self.tb_lbl.pack(anchor="w")
        self.sc_lbl = tk.Label(mbx, text="Scored Balls: 0", fg="#ffb86c", bg="#282a36", font=("Segoe UI", 9, "bold"))
        self.sc_lbl.pack(anchor="w")

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
        self.cnv.create_rectangle(tl[0], tl[1], br[0], br[1], outline="#44475a", fill="#282a36", width=2, tags="static")
        ct = self.w2s(FL / 2.0, FW)
        cb = self.w2s(FL / 2.0, 0)
        self.cnv.create_line(ct[0], ct[1], cb[0], cb[1], fill="#6272a4", dash=(4, 4), width=2, tags="static")

        nz_tl = self.w2s(NZ_X0, NZ_Y1)
        nz_br = self.w2s(NZ_X1, NZ_Y0)
        self.cnv.create_rectangle(nz_tl[0], nz_tl[1], nz_br[0], nz_br[1], outline="#6272a4", fill="#21222c", dash=(2, 2), width=1, tags="static")

        b_tl = self.w2s(BDP_X0, BDP_Y1)
        b_br = self.w2s(BDP_X1, BDP_Y0)
        self.cnv.create_rectangle(b_tl[0], b_tl[1], b_br[0], b_br[1], outline="#8be9fd", fill="#1f2d3d", width=2, tags="static")

        r_tl = self.w2s(RDP_X0, RDP_Y1)
        r_br = self.w2s(RDP_X1, RDP_Y0)
        self.cnv.create_rectangle(r_tl[0], r_tl[1], r_br[0], r_br[1], outline="#ff5555", fill="#3d1f1f", width=2, tags="static")

        hr = self.w2s(RHUB[0], RHUB[1])
        hb = self.w2s(BHUB[0], BHUB[1])
        rad = 0.6 * self.zm
        self.cnv.create_oval(hr[0] - rad, hr[1] - rad, hr[0] + rad, hr[1] + rad, outline="#ff5555", fill="#442222", width=3, tags="static")
        self.cnv.create_oval(hb[0] - rad, hb[1] - rad, hb[0] + rad, hb[1] + rad, outline="#8be9fd", fill="#223344", width=3, tags="static")

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
