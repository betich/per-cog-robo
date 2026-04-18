"""
e-puck_final_slam controller
==============================
Modules (all in one file):
  MotionDetector  — Lucas-Kanade optical flow, dynamic ray mask, display render
  Pose / Odometry
  SparseMap       — dict-backed log-odds occupancy
  PoseGraph       — nodes + edges
  ICP             — scan-to-scan point cloud alignment
  GraphOptimizer  — Gauss-Seidel relaxation back-end
  GraphSLAM       — front-end
  GraphDisplay    — map + graph overlay + HUD on map_display
  Navigator       — frontier-seek + obstacle avoidance
"""

from controller import Robot, Lidar, Motor, PositionSensor, Display, Camera
import math


# ─────────────────────────────────────────────────────────────────────────────
#  Robot constants  (e-puck)
# ─────────────────────────────────────────────────────────────────────────────

WHEEL_RADIUS = 0.0205
WHEEL_BASE   = 0.052
MAX_SPEED    = 6.28
CELL_SIZE    = 0.05


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 1 — MOTION DETECTOR
# ═════════════════════════════════════════════════════════════════════════════

WORK_SIZE = 32   # (changed to 32 from 64) downsample target for Lucas-Kanade


def _gray_full(camera: Camera) -> list:
    w, h  = camera.getWidth(), camera.getHeight()
    image = camera.getImage()
    gray  = [0.0] * (w * h)
    for y in range(h):
        for x in range(w):
            idx = (y * w + x) * 4
            gray[y * w + x] = (0.299 * image[idx + 2]
                               + 0.587 * image[idx + 1]
                               + 0.114 * image[idx])
    return gray


def _downsample(gray: list, sw: int, sh: int, ds: int) -> list:
    out = [0.0] * (ds * ds)
    xr = sw / ds; yr = sh / ds
    for y in range(ds):
        for x in range(ds):
            out[y * ds + x] = gray[min(int(y*yr), sh-1) * sw + min(int(x*xr), sw-1)]
    return out


def _upsample_mask(mask: list, ss: int, dw: int, dh: int) -> list:
    out = [False] * (dw * dh)
    xr = ss / dw; yr = ss / dh
    for y in range(dh):
        for x in range(dw):
            out[y * dw + x] = mask[min(int(y*yr), ss-1) * ss + min(int(x*xr), ss-1)]
    return out


def _sobel(gray: list, w: int, h: int):
    Ix = [0.0] * (w * h); Iy = [0.0] * (w * h)
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            i = y * w + x
            Ix[i] = (-gray[(y-1)*w+(x-1)] + gray[(y-1)*w+(x+1)]
                     -2*gray[y*w+(x-1)]   + 2*gray[y*w+(x+1)]
                     -gray[(y+1)*w+(x-1)] + gray[(y+1)*w+(x+1)]) / 8.0
            Iy[i] = (-gray[(y-1)*w+(x-1)] - 2*gray[(y-1)*w+x] - gray[(y-1)*w+(x+1)]
                     +gray[(y+1)*w+(x-1)] + 2*gray[(y+1)*w+x] + gray[(y+1)*w+(x+1)]) / 8.0
    return Ix, Iy


def _lucas_kanade(prev: list, curr: list, w: int, h: int, wr: int = 2) -> list:
    Ix, Iy = _sobel(prev, w, h)
    It     = [curr[i] - prev[i] for i in range(w * h)]
    mag    = [0.0] * (w * h)
    for y in range(wr, h - wr):
        for x in range(wr, w - wr):
            sxx = syy = sxy = sxt = syt = 0.0
            for dy in range(-wr, wr + 1):
                for dx in range(-wr, wr + 1):
                    k = (y + dy) * w + (x + dx)
                    sxx += Ix[k]*Ix[k]; syy += Iy[k]*Iy[k]; sxy += Ix[k]*Iy[k]
                    sxt += Ix[k]*It[k]; syt += Iy[k]*It[k]
            det = sxx * syy - sxy * sxy
            if abs(det) < 1e-6: continue
            u = (-syy * sxt + sxy * syt) / det
            v = ( sxy * sxt - sxx * syt) / det
            mag[y * w + x] = math.sqrt(u * u + v * v)
    return mag


def _dilate(mask: list, w: int, h: int, r: int = 1) -> list:
    out = [False] * (w * h)
    for y in range(h):
        for x in range(w):
            if mask[y * w + x]:
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            out[ny * w + nx] = True
    return out


class MotionDetector:
    """
    Pipeline per timestep:
      gray(256x256) → downsample(64x64) → LK flow → threshold → dilate
      → upsample mask(256x256) → query per LiDAR ray
    """
    FLOW_THRESHOLD = 1.5
    DILATE_RADIUS  = 2

    def __init__(self, cam_w: int, cam_h: int, cam_fov: float):
        self.cam_w = cam_w; self.cam_h = cam_h; self.fov = cam_fov
        self.prev_small = None
        self.mask_full  = [False] * (cam_w * cam_h)
        self.gray_full  = [0.0]   * (cam_w * cam_h)
        self.ready      = False
        self._dist_moved = 0.0
        self._prev_px    = 0.0
        self._prev_py    = 0.0

    def update(self, camera: Camera, robot_x: float = 0.0, robot_y: float = 0.0) -> None:
        self._dist_moved += math.hypot(robot_x - self._prev_px,
                                       robot_y - self._prev_py)
        self._prev_px = robot_x; self._prev_py = robot_y

        gray           = _gray_full(camera)
        self.gray_full = gray

        if self._dist_moved < 0.001 and self.ready:
            return

        self._dist_moved = 0.0
    
        gray           = _gray_full(camera)
        self.gray_full = gray
        small          = _downsample(gray, self.cam_w, self.cam_h, WORK_SIZE)
        if self.prev_small is None:
            self.prev_small = small
            return
        flow            = _lucas_kanade(self.prev_small, small, WORK_SIZE, WORK_SIZE)
        raw             = [flow[i] > self.FLOW_THRESHOLD
                           for i in range(WORK_SIZE * WORK_SIZE)]
        mask_small      = _dilate(raw, WORK_SIZE, WORK_SIZE, self.DILATE_RADIUS)
        self.mask_full  = _upsample_mask(mask_small, WORK_SIZE,
                                         self.cam_w, self.cam_h)
        self.prev_small = small
        self.ready      = True

    def is_dynamic(self, ray_angle: float) -> bool:
        if not self.ready or abs(ray_angle) > self.fov / 2:
            return False
        col = max(0, min(self.cam_w - 1,
                         int((ray_angle / self.fov + 0.5) * self.cam_w)))
        return self.mask_full[(self.cam_h // 2) * self.cam_w + col]

    def render(self, disp: Display) -> None:
        """Red = dynamic, gray = static. Call every few steps."""
        if not self.ready:
            return
        
        w, h = self.cam_w, self.cam_h
        for y in range(h):
            for x in range(w):
                i = y * w + x
                if self.mask_full[i]:
                    disp.setColor(0xff0000)
                else:
                    v = int(self.gray_full[i])
                    disp.setColor((v << 16) | (v << 8) | v)
                disp.drawPixel(x, y)

    def stats(self) -> str:
        n = sum(self.mask_full)
        t = self.cam_w * self.cam_h
        return f"motion:{n}/{t}px({100*n/t:.1f}%)"


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 2 — POSE + ODOMETRY
# ═════════════════════════════════════════════════════════════════════════════

class Pose:
    __slots__ = ('x', 'y', 'theta')

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x; self.y = y; self.theta = theta

    def copy(self): return Pose(self.x, self.y, self.theta)

    @staticmethod
    def wrap(theta):
        while theta >  math.pi: theta -= 2 * math.pi
        while theta < -math.pi: theta += 2 * math.pi
        return theta


def odometry_update(pose: Pose, prev_l, prev_r, cur_l, cur_r) -> Pose:
    dl = (cur_l - prev_l) * WHEEL_RADIUS
    dr = (cur_r - prev_r) * WHEEL_RADIUS
    dc = (dl + dr) / 2.0
    dtheta = (dr - dl) / WHEEL_BASE
    p = pose.copy()
    p.x    += dc * math.cos(pose.theta + dtheta / 2.0)
    p.y    += dc * math.sin(pose.theta + dtheta / 2.0)
    p.theta = Pose.wrap(pose.theta + dtheta)
    return p


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 3 — SPARSE MAP
# ═════════════════════════════════════════════════════════════════════════════

class SparseMap:
    L_OCC       =  0.6
    L_FREE_NEAR = -0.6
    L_FREE_FAR  = -0.3
    L_MIN       = -4.0
    L_MAX       =  4.0
    OCC_THRESH  =  1.2
    FREE_THRESH = -1.2

    def __init__(self):
        self.cells: dict = {}

    def _key(self, wx, wy):
        return (int(math.floor(wx / CELL_SIZE)),
                int(math.floor(wy / CELL_SIZE)))

    def get(self, ix, iy): return self.cells.get((ix, iy), 0.0)

    def _set(self, ix, iy, val):
        self.cells[(ix, iy)] = max(self.L_MIN, min(self.L_MAX, val))

    def is_free(self, wx, wy) -> bool:
        return self.get(*self._key(wx, wy)) < self.FREE_THRESH

    def is_occupied(self, wx, wy) -> bool:
        return self.get(*self._key(wx, wy)) > self.OCC_THRESH

    def update_ray(self, rx, ry, world_angle, hit_dist, max_range,
                   is_dynamic=False):
        if is_dynamic or hit_dist < 0.05:
            return
        is_hit  = hit_dist < max_range - 0.08
        trace_d = hit_dist if is_hit else (max_range - 0.08)
        end_x   = rx + trace_d * math.cos(world_angle)
        end_y   = ry + trace_d * math.sin(world_angle)
        x0, y0  = self._key(rx, ry)
        x1, y1  = self._key(end_x, end_y)
        dx = abs(x1-x0); dy = abs(y1-y0)
        sx = 1 if x0 < x1 else -1; sy = 1 if y0 < y1 else -1
        err = dx - dy; cx, cy = x0, y0; step = 0
        for _ in range(dx + dy + 1):
            if cx == x1 and cy == y1:
                if is_hit: self._set(cx, cy, self.get(cx, cy) + self.L_OCC)
                break
            lf = self.L_FREE_NEAR if step < 6 else self.L_FREE_FAR
            self._set(cx, cy, self.get(cx, cy) + lf)
            step += 1
            e2 = 2 * err
            if e2 > -dy: err -= dy; cx += sx
            if e2 <  dx: err += dx; cy += sy

    def rebuild(self, nodes, scans, lidar_params, motion_masks=None):
        self.cells.clear()
        start_angle, ang_step, max_range, n_rays = lidar_params
        for node in nodes:
            scan = scans.get(node.id)
            if scan is None: continue
            mask = motion_masks.get(node.id) if motion_masks else None
            for i, r in enumerate(scan):
                if math.isnan(r) or math.isinf(r) or r <= 0: continue
                is_dyn = bool(mask[i]) if (mask and i < len(mask)) else False
                world_angle = node.pose.theta + start_angle + i * ang_step
                self.update_ray(node.pose.x, node.pose.y,
                                world_angle, r, max_range, is_dyn)

    def decay_occupied(self):
        for k, v in list(self.cells.items()):
            if v > 0.1: self.cells[k] = max(0.0, v - 0.05)


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 4 — POSE GRAPH
# ═════════════════════════════════════════════════════════════════════════════

class PoseNode:
    __slots__ = ('id', 'pose')
    def __init__(self, node_id, pose):
        self.id = node_id; self.pose = pose.copy()

class PoseEdge:
    __slots__ = ('from_id', 'to_id', 'dx', 'dy', 'dtheta', 'is_loop')
    def __init__(self, from_id, to_id, dx, dy, dtheta, is_loop=False):
        self.from_id = from_id; self.to_id = to_id
        self.dx = dx; self.dy = dy; self.dtheta = dtheta; self.is_loop = is_loop

class PoseGraph:
    def __init__(self):
        self.nodes: list = []; self.edges: list = []; self._id_map: dict = {}

    def add_node(self, pose) -> PoseNode:
        n = PoseNode(len(self.nodes), pose)
        self.nodes.append(n); self._id_map[n.id] = n; return n

    def add_edge(self, from_id, to_id, dx, dy, dtheta, is_loop=False):
        self.edges.append(PoseEdge(from_id, to_id, dx, dy, dtheta, is_loop))

    def get_node(self, nid) -> PoseNode: return self._id_map[nid]

    @property
    def loop_count(self): return sum(1 for e in self.edges if e.is_loop)


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 5 — ICP
# ═════════════════════════════════════════════════════════════════════════════

def _frange(start, stop, step):
    v = start
    while v <= stop + 1e-9: yield v; v += step


class ICP:
    @staticmethod
    def build_cloud(ranges, n_rays, start_angle, ang_step, max_range, pose):
        cloud = []
        for i in range(0, n_rays, 4):
            r = ranges[i]
            if math.isnan(r) or math.isinf(r) or r < 0.06 or r > max_range - 0.08:
                continue
            angle = pose.theta + start_angle + i * ang_step
            cloud.append((pose.x + r * math.cos(angle),
                          pose.y + r * math.sin(angle)))
        return cloud

    @staticmethod
    def cloud_error(A, B):
        if not A or not B: return 1e9
        total = 0.0
        for ax, ay in A:
            best = 1e9
            for bx, by in B:
                d = (ax-bx)**2 + (ay-by)**2
                if d < best: best = d
            total += best
        return total / len(A)

    @staticmethod
    def transform_cloud(cloud, dx, dy, dth):
        c, s = math.cos(dth), math.sin(dth)
        return [(c*x - s*y + dx, s*x + c*y + dy) for x, y in cloud]

    @staticmethod
    def match(cloud_curr, cloud_ref, max_trans=0.04, max_rot=0.052):
        best_err = ICP.cloud_error(cloud_curr, cloud_ref)
        best_dx = best_dy = best_dth = 0.0
        dth_step = max_rot / 3; dxy_step = max_trans / 4
        for dth in _frange(-max_rot, max_rot, dth_step):
            for dx in _frange(-max_trans, max_trans, dxy_step):
                for dy in _frange(-max_trans, max_trans, dxy_step):
                    tmp = ICP.transform_cloud(cloud_curr, dx, dy, dth)
                    e   = ICP.cloud_error(tmp, cloud_ref)
                    if e < best_err:
                        best_err = e; best_dx = dx; best_dy = dy; best_dth = dth
        for dth in _frange(best_dth-dth_step, best_dth+dth_step, dth_step/3):
            for dx in _frange(best_dx-dxy_step, best_dx+dxy_step, dxy_step/3):
                for dy in _frange(best_dy-dxy_step, best_dy+dxy_step, dxy_step/3):
                    tmp = ICP.transform_cloud(cloud_curr, dx, dy, dth)
                    e   = ICP.cloud_error(tmp, cloud_ref)
                    if e < best_err:
                        best_err = e; best_dx = dx; best_dy = dy; best_dth = dth
        return best_dx, best_dy, best_dth, best_err


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 6 — GRAPH OPTIMIZER
# ═════════════════════════════════════════════════════════════════════════════

class GraphOptimizer:
    MAX_ITER     = 20
    LR           = 0.3
    LOOP_LR      = 0.5
    CONVERGE_EPS = 1e-4

    @staticmethod
    def optimize(graph: PoseGraph):
        if len(graph.nodes) < 2: return
        for _ in range(GraphOptimizer.MAX_ITER):
            max_c = 0.0
            for edge in graph.edges:
                na = graph.get_node(edge.from_id)
                nb = graph.get_node(edge.to_id)
                err_x  = edge.dx     - (nb.pose.x     - na.pose.x)
                err_y  = edge.dy     - (nb.pose.y     - na.pose.y)
                err_th = Pose.wrap(edge.dtheta
                                   - Pose.wrap(nb.pose.theta - na.pose.theta))
                lr = GraphOptimizer.LOOP_LR if edge.is_loop else GraphOptimizer.LR
                if na.id > 0:
                    na.pose.x     -= lr * 0.5 * err_x
                    na.pose.y     -= lr * 0.5 * err_y
                    na.pose.theta  = Pose.wrap(na.pose.theta - lr * 0.5 * err_th)
                nb.pose.x     += lr * 0.5 * err_x
                nb.pose.y     += lr * 0.5 * err_y
                nb.pose.theta  = Pose.wrap(nb.pose.theta + lr * 0.5 * err_th)
                max_c = max(max_c, abs(err_x) + abs(err_y) + abs(err_th))
            if max_c < GraphOptimizer.CONVERGE_EPS: break


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 7 — GRAPH SLAM FRONT-END
# ═════════════════════════════════════════════════════════════════════════════

class GraphSLAM:
    NODE_DIST_M  = 0.10
    LOOP_RADIUS  = 0.5
    LOOP_ERR_MAX = 0.015
    OPT_EVERY    = 10

    def __init__(self):
        self.graph    = PoseGraph()
        self.occ_map  = SparseMap()
        self.scans:   dict = {}
        self.clouds:  dict = {}
        self.masks:   dict = {}
        self.lidar_params       = None
        self.prev_node          = None
        self.dist_since_node    = 0.0
        self.needs_rebuild      = False

    def init_lidar(self, start_angle, ang_step, max_range, n_rays):
        self.lidar_params = (start_angle, ang_step, max_range, n_rays)

    def update(self, pose: Pose, ranges, motion_mask_flags=None) -> bool:
        if self.lidar_params is None: return False
        start_angle, ang_step, max_range, n_rays = self.lidar_params

        if self.prev_node is not None:
            self.dist_since_node += math.hypot(
                pose.x - self.prev_node.pose.x,
                pose.y - self.prev_node.pose.y)

        if self.prev_node is None or self.dist_since_node >= self.NODE_DIST_M:
            self._add_node(pose, ranges, motion_mask_flags,
                           start_angle, ang_step, max_range, n_rays)
            self.dist_since_node = 0.0

        if ranges is not None:
            for i in range(n_rays):
                r = ranges[i]
                if math.isnan(r) or math.isinf(r) or r <= 0: continue
                is_dyn = (motion_mask_flags[i]
                          if motion_mask_flags and i < len(motion_mask_flags)
                          else False)
                world_angle = pose.theta + start_angle + i * ang_step
                self.occ_map.update_ray(pose.x, pose.y, world_angle,
                                        r, max_range, is_dyn)

        n_nodes = len(self.graph.nodes)
        if n_nodes > 1 and n_nodes % self.OPT_EVERY == 0 and self.needs_rebuild:
            GraphOptimizer.optimize(self.graph)
            self.occ_map.rebuild(self.graph.nodes, self.scans,
                                 self.lidar_params, self.masks)
            self.needs_rebuild = False
            return True
        return False

    def _add_node(self, pose, ranges, mask_flags,
                  start_angle, ang_step, max_range, n_rays):
        node = self.graph.add_node(pose)
        if ranges is not None:
            self.scans[node.id]  = [ranges[i] for i in range(n_rays)]
            self.clouds[node.id] = ICP.build_cloud(
                ranges, n_rays, start_angle, ang_step, max_range, pose)
            if mask_flags:
                self.masks[node.id] = list(mask_flags)

        if self.prev_node is not None:
            dx     = pose.x     - self.prev_node.pose.x
            dy     = pose.y     - self.prev_node.pose.y
            dtheta = Pose.wrap(pose.theta - self.prev_node.pose.theta)
            prev_c = self.clouds.get(self.prev_node.id, [])
            curr_c = self.clouds.get(node.id, [])
            if len(prev_c) >= 10 and len(curr_c) >= 10:
                idx, idy, idth, err = ICP.match(curr_c, prev_c)
                be = ICP.cloud_error(curr_c, prev_c)
                if (err < be * 0.85 and abs(idx) < 0.035
                        and abs(idy) < 0.035 and abs(idth) < 0.045):
                    dx += idx; dy += idy; dtheta = Pose.wrap(dtheta + idth)
            self.graph.add_edge(self.prev_node.id, node.id, dx, dy, dtheta)
            self._check_loop_closure(node, curr_c)

        self.prev_node     = node
        self.needs_rebuild = True

    def _check_loop_closure(self, new_node, new_cloud):
        if len(new_cloud) < 10: return
        for old_node in self.graph.nodes[:-5]:
            if math.hypot(new_node.pose.x - old_node.pose.x,
                          new_node.pose.y - old_node.pose.y) > self.LOOP_RADIUS:
                continue
            old_cloud = self.clouds.get(old_node.id, [])
            if len(old_cloud) < 10: continue
            if ICP.cloud_error(new_cloud, old_cloud) > 0.1: continue
            cdx, cdy, cdth, err = ICP.match(new_cloud, old_cloud,
                                             max_trans=self.LOOP_RADIUS,
                                             max_rot=0.5)
            if err < self.LOOP_ERR_MAX:
                self.graph.add_edge(
                    new_node.id, old_node.id,
                    old_node.pose.x + cdx - new_node.pose.x,
                    old_node.pose.y + cdy - new_node.pose.y,
                    Pose.wrap(old_node.pose.theta + cdth - new_node.pose.theta),
                    is_loop=True)
                print(f"[SLAM] Loop closure: node {new_node.id}→{old_node.id}"
                      f"  err={err:.4f}")
                self.needs_rebuild = True
                break


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 8 — GRAPH DISPLAY
# ═════════════════════════════════════════════════════════════════════════════

class GraphDisplay:
    HUD_H         = 24
    SCALE_M       = 80.0    # pixels per metre — increase if arena looks tiny
    COL_BG        = 0x0a0a0a
    COL_UNKNOWN   = 0x111111
    COL_NODE      = 0xaaaaaa
    COL_NODE_CUR  = 0xffffff
    COL_EDGE_ODO  = 0x666666
    COL_EDGE_LOOP = 0x00ddcc
    COL_HUD_BG    = 0x0a0f0a
    COL_HUD_TXT   = 0x00cc44

    def __init__(self, disp: Display):
        self.disp = disp
        self.dw   = disp.getWidth()
        self.dh   = disp.getHeight() - self.HUD_H
        self.vx   = self.vy = 0.0

    def _w2p(self, wx, wy):
        return (int((wx - self.vx) * self.SCALE_M + self.dw / 2),
                int(self.dh / 2 - (wy - self.vy) * self.SCALE_M))

    def _ok(self, px, py): return 0 <= px < self.dw and 0 <= py < self.dh

    def render(self, slam: GraphSLAM, current_pose: Pose):
        d = self.disp
        self.vx = current_pose.x; self.vy = current_pose.y
        d.setColor(self.COL_BG)
        d.fillRectangle(0, 0, self.dw, self.dh + self.HUD_H)

        # Layer 1 — occupancy cells
        cpx = max(1, int(CELL_SIZE * self.SCALE_M))
        for (ix, iy), lo in slam.occ_map.cells.items():
            px, py = self._w2p(ix * CELL_SIZE + CELL_SIZE / 2,
                               iy * CELL_SIZE + CELL_SIZE / 2)
            if not self._ok(px, py): continue
            if lo > slam.occ_map.OCC_THRESH:
                t = min(1.0, (lo - 1.2) / 2.8)
                color = (int(80 + 120 * t) << 16) | 0x1100
            elif lo < slam.occ_map.FREE_THRESH:
                t = min(1.0, (-lo - 1.2) / 2.8)
                color = int(40 + 100 * t) << 8
            else:
                color = self.COL_UNKNOWN
            d.setColor(color); d.fillRectangle(px, py, cpx, cpx)

        # Layer 2a — odometry edges
        for edge in slam.graph.edges:
            if edge.is_loop: continue
            na = slam.graph.get_node(edge.from_id)
            nb = slam.graph.get_node(edge.to_id)
            ax, ay = self._w2p(na.pose.x, na.pose.y)
            bx, by = self._w2p(nb.pose.x, nb.pose.y)
            if self._ok(ax, ay) or self._ok(bx, by):
                d.setColor(self.COL_EDGE_ODO); d.drawLine(ax, ay, bx, by)

        # Layer 2b — loop closure edges (cyan)
        for edge in slam.graph.edges:
            if not edge.is_loop: continue
            na = slam.graph.get_node(edge.from_id)
            nb = slam.graph.get_node(edge.to_id)
            ax, ay = self._w2p(na.pose.x, na.pose.y)
            bx, by = self._w2p(nb.pose.x, nb.pose.y)
            if self._ok(ax, ay) or self._ok(bx, by):
                d.setColor(self.COL_EDGE_LOOP); d.drawLine(ax, ay, bx, by)

        # Layer 2c — pose nodes
        for node in slam.graph.nodes:
            px, py = self._w2p(node.pose.x, node.pose.y)
            if not self._ok(px, py): continue
            d.setColor(self.COL_NODE); d.fillOval(px - 2, py - 2, 4, 4)

        # Current robot dot + heading line
        cx, cy = self._w2p(current_pose.x, current_pose.y)
        if self._ok(cx, cy):
            d.setColor(self.COL_NODE_CUR); d.fillOval(cx - 4, cy - 4, 8, 8)
            hx = cx + int(12 * math.cos(current_pose.theta))
            hy = cy - int(12 * math.sin(current_pose.theta))
            d.drawLine(cx, cy, hx, hy)

        # HUD
        hud_y = self.dh
        d.setColor(self.COL_HUD_BG); d.fillRectangle(0, hud_y, self.dw, self.HUD_H)
        d.setColor(0x2a3a2a); d.drawLine(0, hud_y, self.dw, hud_y)
        d.setColor(self.COL_HUD_TXT)
        d.drawText(f"n:{len(slam.graph.nodes)} e:{len(slam.graph.edges)}"
                   f" lc:{slam.graph.loop_count}", 4, hud_y + 6)


# ═════════════════════════════════════════════════════════════════════════════
#  MODULE 9 — NAVIGATOR
#  SEEK  — steer toward nearest frontier cell
#  AVOID — spin away from obstacle for a timed number of steps
#  WANDER — no frontiers found, spin slowly
# ═════════════════════════════════════════════════════════════════════════════

class Navigator:
    OBS_DANGER = 0.10   # m — triggers avoidance spin
    OBS_WARN   = 0.18   # m — slows linear speed
    K_ANG      = 3.0    # proportional heading gain
    K_LIN      = 0.8    # linear speed fraction of MAX_SPEED

    def __init__(self):
        self.state       = "SEEK"
        self.goal_x      = 0.0
        self.goal_y      = 0.0
        self.has_goal    = False
        self.avoid_timer = 0
        self.avoid_dir   = 1.0
        self._step       = 0

    def _sector_min(self, ranges, nr, sa, step, lo, hi, mr):
        mn = mr
        for i in range(nr):
            r = ranges[i]
            if math.isnan(r) or math.isinf(r) or r <= 0: continue
            a = sa + i * step
            if lo <= a <= hi and r < mn: mn = r
        return mn

    def _find_frontier(self, occ_map: SparseMap, pose: Pose):
        """
        Frontier = free cell with at least one unknown 4-neighbour.
        Returns world coords (gx, gy) of the closest one, or (None, None).
        """
        if not occ_map.cells:
            return None, None
        rx = int(math.floor(pose.x / CELL_SIZE))
        ry = int(math.floor(pose.y / CELL_SIZE))
        frontiers = []
        for (ix, iy), lo in occ_map.cells.items():
            if lo >= occ_map.FREE_THRESH: continue
            for ddx, ddy in ((1,0),(-1,0),(0,1),(0,-1)):
                if (ix+ddx, iy+ddy) not in occ_map.cells:
                    frontiers.append((ix, iy)); break
        if not frontiers:
            return None, None
        best = min(frontiers, key=lambda c: (c[0]-rx)**2 + (c[1]-ry)**2)
        return (best[0] * CELL_SIZE + CELL_SIZE / 2,
                best[1] * CELL_SIZE + CELL_SIZE / 2)

    def update(self, pose: Pose, occ_map: SparseMap,
               ranges, nr, sa, step, mr):
        """Returns (left_v, right_v, state_name). Call every timestep."""
        self._step += 1
        if ranges is None:
            return 0.0, 0.0, "WAIT"

        front = self._sector_min(ranges, nr, sa, step, -math.pi/5, math.pi/5, mr)
        left  = self._sector_min(ranges, nr, sa, step,  math.pi/5, math.pi*2/3, mr)
        right = self._sector_min(ranges, nr, sa, step, -math.pi*2/3, -math.pi/5, mr)

        # ── AVOID: count down, then back to SEEK ──────────────────────────
        if self.state == "AVOID":
            if self.avoid_timer > 0:
                self.avoid_timer -= 1
                spd = MAX_SPEED * 0.45
                return spd * self.avoid_dir, -spd * self.avoid_dir, "AVOID"
            self.state    = "SEEK"
            self.has_goal = False

        # ── Obstacle → enter AVOID ────────────────────────────────────────
        if front < self.OBS_DANGER:
            self.avoid_dir   = -1.0 if left < right else 1.0
            self.avoid_timer = 20 + (self._step % 30)
            self.state       = "AVOID"
            spd = MAX_SPEED * 0.45
            return spd * self.avoid_dir, -spd * self.avoid_dir, "AVOID"

        # ── Refresh frontier goal ─────────────────────────────────────────
        if (not self.has_goal or
                math.hypot(pose.x - self.goal_x,
                           pose.y - self.goal_y) < CELL_SIZE * 1.5):
            gx, gy = self._find_frontier(occ_map, pose)
            if gx is None:
                spd = MAX_SPEED * 0.3
                return spd, -spd, "WANDER"
            self.goal_x = gx; self.goal_y = gy; self.has_goal = True

        # ── Proportional heading control ──────────────────────────────────
        err = Pose.wrap(math.atan2(self.goal_y - pose.y,
                                   self.goal_x - pose.x) - pose.theta)
        scale = (((front - self.OBS_DANGER) /
                  (self.OBS_WARN  - self.OBS_DANGER)) * 0.6 + 0.1
                 if front < self.OBS_WARN else 1.0)
        lin = min(MAX_SPEED * self.K_LIN * scale, MAX_SPEED * 0.85)
        ang = self.K_ANG * err
        lv  = lin - ang * WHEEL_BASE / 2
        rv  = lin + ang * WHEEL_BASE / 2
        mx  = max(abs(lv), abs(rv))
        if mx > MAX_SPEED: lv /= mx / MAX_SPEED; rv /= mx / MAX_SPEED
        return lv, rv, "SEEK"


# ═════════════════════════════════════════════════════════════════════════════
#  MAIN CONTROLLER
# ═════════════════════════════════════════════════════════════════════════════

robot    = Robot()
timestep = int(robot.getBasicTimeStep())

# ── LiDAR ─────────────────────────────────────────────────────────────────────
lidar: Lidar = robot.getDevice("lidar")
if lidar is None:
    lidar = robot.getDevice("LDS-01")
lidar.enable(timestep)
lidar.enablePointCloud()

N_RAYS      = lidar.getHorizontalResolution()
LIDAR_MAX_R = lidar.getMaxRange()
LIDAR_FOV   = lidar.getFov()
START_ANGLE = -LIDAR_FOV / 2.0
ANG_STEP    = LIDAR_FOV / max(N_RAYS - 1, 1)
print(f"[INIT] LiDAR {N_RAYS}rays FOV={math.degrees(LIDAR_FOV):.0f}deg maxR={LIDAR_MAX_R}m")

# ── Motors + encoders ─────────────────────────────────────────────────────────
lm: Motor = robot.getDevice("left wheel motor")
rm: Motor = robot.getDevice("right wheel motor")
lm.setPosition(float('inf')); rm.setPosition(float('inf'))
lm.setVelocity(0); rm.setVelocity(0)

le: PositionSensor = robot.getDevice("left wheel sensor")
re: PositionSensor = robot.getDevice("right wheel sensor")
le.enable(timestep); re.enable(timestep)

# ── Camera ────────────────────────────────────────────────────────────────────
camera: Camera = robot.getDevice("camera")
camera.enable(timestep)
CAM_W   = camera.getWidth()
CAM_H   = camera.getHeight()
CAM_FOV = camera.getFov()
print(f"[INIT] Camera {CAM_W}x{CAM_H} FOV={math.degrees(CAM_FOV):.0f}deg")

# ── Displays ──────────────────────────────────────────────────────────────────
# map_display           — set to 400×424 in Webots scene tree
# motion_vision_display — set to same WxH as camera
map_disp    = robot.getDevice("map_display")
motion_disp = robot.getDevice("motion_vision_display")
if not map_disp:    print("[INIT] WARNING: no 'map_display'")
if not motion_disp: print("[INIT] WARNING: no 'motion_vision_display'")

graph_display = GraphDisplay(map_disp) if map_disp else None

# ── Module instances ──────────────────────────────────────────────────────────
detector = MotionDetector(CAM_W, CAM_H, CAM_FOV)

slam = GraphSLAM()
slam.init_lidar(START_ANGLE, ANG_STEP, LIDAR_MAX_R, N_RAYS)

nav  = Navigator()
pose = Pose()

prev_l = prev_r = 0.0
first_step  = True
step_count  = 0
DETECTOR_EVERY = 3
DISP_EVERY  = 30
DECAY_EVERY = 20
LOG_EVERY   = 100

print("[INIT] All modules ready — starting")

# ── Main loop ─────────────────────────────────────────────────────────────────
while robot.step(timestep) != -1:
    step_count += 1

    # 1. Odometry ──────────────────────────────────────────────────────────────
    cur_l = le.getValue(); cur_r = re.getValue()
    if not first_step:
        pose = odometry_update(pose, prev_l, prev_r, cur_l, cur_r)
    else:
        first_step = False
    prev_l = cur_l; prev_r = cur_r

    # 2. Motion detection + render to motion_vision_display ───────────────────
    if step_count % DETECTOR_EVERY  == 0:
        detector.update(camera, pose.x, pose.y)
        
    if motion_disp and step_count % DISP_EVERY == 0:
        detector.render(motion_disp)    # red=dynamic  gray=static

    # 3. LiDAR + per-ray dynamic flags ─────────────────────────────────────────
    ranges = lidar.getRangeImage()
    motion_ray_flags = None
    if detector.ready and ranges:
        motion_ray_flags = [
            detector.is_dynamic(START_ANGLE + i * ANG_STEP)
            for i in range(N_RAYS)
        ]

    # 4. Graph SLAM ────────────────────────────────────────────────────────────
    if ranges:
        optimized = slam.update(pose, ranges, motion_ray_flags)
        if optimized:
            print(f"[SLAM] optimized  step={step_count}"
                  f"  nodes={len(slam.graph.nodes)}"
                  f"  loops={slam.graph.loop_count}")

    # 5. Passive map decay ─────────────────────────────────────────────────────
    if step_count % DECAY_EVERY == 0:
        slam.occ_map.decay_occupied()

    # 6. Navigation ────────────────────────────────────────────────────────────
    lv, rv, nav_mode = nav.update(
        pose, slam.occ_map, ranges,
        N_RAYS, START_ANGLE, ANG_STEP, LIDAR_MAX_R)
    lm.setVelocity(lv)
    rm.setVelocity(rv)

    # 7. Render map display ────────────────────────────────────────────────────
    if graph_display and step_count % DISP_EVERY == 0:
        graph_display.render(slam, pose)

    # 8. Periodic log ──────────────────────────────────────────────────────────
    if step_count % LOG_EVERY == 0:
        print(f"[{step_count:5d}] "
              f"pose=({pose.x:.2f},{pose.y:.2f},{math.degrees(pose.theta):.0f}d) "
              f"cells={len(slam.occ_map.cells)} "
              f"nodes={len(slam.graph.nodes)} "
              f"loops={slam.graph.loop_count} "
              f"nav={nav_mode} | {detector.stats()}")