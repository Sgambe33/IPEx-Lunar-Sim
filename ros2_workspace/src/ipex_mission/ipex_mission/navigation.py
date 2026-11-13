"""
Grid A* path planning on costmap and pure pursuit controller.
"""
import heapq
import math
import numpy as np
from .geometry import Point, Pose, wrap


class CostMap:
    """Costmap with obstacles inflated by the rover size."""

    def __init__(self, half_size: float = 25.0, resolution: float = 0.2, inflation: float = 0.5) -> None:
        self.res: float = resolution                                    # size in m per cell
        self.x0: float = -half_size                                     # x coordinate of bottom left cell
        self.y0: float = -half_size                                     # y coordinate of bottom left cell
        self.n: int = int(round(2 * half_size / resolution)) + 1        # number of cells on each side
        self.inflation: float = inflation                               # how much to add to obstacles
        self.occ: np.ndarray = np.zeros((self.n, self.n), dtype=bool)   # n x n boolean matrix (init with all False)
        xs = self.x0 + np.arange(self.n) * resolution                   # sequence of all cell centers (x coordinate)
        X, Y = np.meshgrid(xs, xs)                                     
        self.X: np.ndarray = X                                          # matrix with all cells center x coordinate
        self.Y: np.ndarray = Y                                          # matrix with all cells center y coordinate

    def add_circle(self, x:float, y:float, radius:float):
        """Adds a circle obstacle."""
        r = radius + self.inflation
        self.occ |= (self.X - x) ** 2 + (self.Y - y) ** 2 <= r * r # obstacle insertion via broadcasting

    def add_mask(self, mask):
        """Applies a mask (boolean matrix) to the costmap."""
        self.occ |= mask

    def to_index(self, x, y):
        """Meters coordinates to cell index."""
        return int(round((y - self.y0) / self.res)), int(round((x - self.x0) / self.res))

    def to_point(self, iy, ix):
        """Cell index to meters coordinates."""
        return self.x0 + ix * self.res, self.y0 + iy * self.res

    def inside(self, iy, ix):
        """Checks if indexes iy and ix are inside costmap."""
        return 0 <= iy < self.n and 0 <= ix < self.n

    def free(self, iy, ix):
        """Checks if cell [iy,ix] is free (set to False)."""
        return self.inside(iy, ix) and not self.occ[iy, ix]

    def free_point(self, x, y):
        """Checks if cell containing (x,y) is free (set to False)."""
        iy, ix = self.to_index(x, y)
        return self.free(iy, ix)

    def nearest_free(self, iy, ix, max_r=40):
        """Finds nearest free cell given cell indexes."""
        if self.free(iy, ix):
            return iy, ix
        for r in range(1, max_r):
            best = None
            for dy in range(-r, r + 1):
                for dx in (-r, r) if abs(dy) != r else range(-r, r + 1):
                    if self.free(iy + dy, ix + dx):
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, iy + dy, ix + dx)
            if best:
                return best[1], best[2]
        return None

    def line_free(self, a, b):
        """Check if line from a to b is free from obstacles."""
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(2, int(dist / (self.res * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            if not self.free_point(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])):
                return False
        return True

    def plan(self, start: Point, goal: Point) -> list[Point] | None:
        """A* on the grid in 8 directions, from start to goal. Returns the path waypoints or None."""
        # a node is a cell (iy, ix);
        #   g(n) = cost of the best path found so far from the start to n
        #   h(n) = estimated cost from n to the goal
        #   f(n) = g(n) + h(n) the order in which nodes are expanded

        start_iy, start_ix = self.to_index(start[0], start[1])
        goal_iy, goal_ix = self.to_index(goal[0], goal[1])
        start_node = self.nearest_free(start_iy, start_ix)
        goal_node = self.nearest_free(goal_iy, goal_ix)

        if start_node is None or goal_node is None:
            return None
        if start_node == goal_node:
            return [start, goal]

        #(d_iy, d_ix, cost)
        successors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

        def h(node):
            return math.hypot(node[0] - goal_node[0], node[1] - goal_node[1])

        open_list = [(h(start_node), 0.0, start_node)] #nodes found but not explored yet
        closed_list = set() #nodes already explored
        g = {start_node: 0.0}
        parent = {start_node: None}

        while open_list:
            _, g_q, q = heapq.heappop(open_list)
            if q in closed_list:
                continue
            closed_list.add(q)
            if q == goal_node:
                break
            for d_iy, d_ix, cost in successors:
                succ_iy = q[0] + d_iy
                succ_ix = q[1] + d_ix
                succ = (succ_iy, succ_ix)
                if succ in closed_list or not self.free(succ_iy, succ_ix):
                    continue
                g_succ = g_q + cost
                if g_succ >= g.get(succ, float("inf")):
                    continue
                g[succ] = g_succ
                parent[succ] = q
                heapq.heappush(open_list, (g_succ + h(succ), g_succ, succ))

        if goal_node not in parent:
            return None

        cells = []
        node = goal_node
        while node is not None:
            cells.append(node)
            node = parent[node]
        cells.reverse()

        waypoints = [self.to_point(iy, ix) for iy, ix in cells]
        if self.free_point(start[0], start[1]):
            waypoints[0] = start
        if self.free_point(goal[0], goal[1]):
            waypoints[-1] = goal
        return self.shortcut(waypoints)

    def shortcut(self, pts):
        """Tries to find a more direct path from start to goal by checking if shortcuts can be taken. Avoids zig-zagging."""
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.line_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out


class PurePursuit:
    def __init__(self, lookahead: float = 0.6, v_max: float = 0.25, w_max: float = 0.6, sharp_turn_above_deg: float = 50.0, sharp_turn_speed: float = 0.05) -> None:
        self.lookahead: float = lookahead                                   # lookahead point distance from rover
        self.v_max: float = v_max                                           # max linear speed m/S
        self.w_max: float = w_max                                           # max angular speed rad/S
        self.sharp_turn_above: float = math.radians(sharp_turn_above_deg)   # max direction error above which rover turns in place
        self.sharp_turn_speed: float = sharp_turn_speed                     # max fwd speed when turning in place

    def target_point(self, pose:Pose, path:list[Point]):
        """Returns the lookahead point coordinates and its distance from current pose."""
        x, y, _ = pose
        best_i = 0
        best_d = float("inf")
        best_t = 0.0

        # enumerate each path segment
        for i, (a, b) in enumerate(zip(path, path[1:])):
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            dist_squared = dx * dx + dy * dy
            t = 0.0 if dist_squared < 1e-12 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / dist_squared))
            d = math.hypot(a[0] + t * dx - x, a[1] + t * dy - y)
            if d < best_d:
                best_i = i
                best_d = d
                best_t = t

        # walk forward lookahead along the path from the projection
        a = path[best_i]
        b = path[best_i + 1] if best_i + 1 < len(path) else path[best_i]
        px = a[0] + best_t * (b[0] - a[0])
        py = a[1] + best_t * (b[1] - a[1])

        remaining = self.lookahead
        cur = (px, py)
        for k in range(best_i + 1, len(path)):
            seg = math.hypot(path[k][0] - cur[0], path[k][1] - cur[1])
            if seg >= remaining:
                t = remaining / seg
                return (cur[0] + t * (path[k][0] - cur[0]), cur[1] + t * (path[k][1] - cur[1])), best_d
            remaining -= seg
            cur = path[k]
        return path[-1], best_d

    def command(self, pose:Pose, target:Point, reverse=False, v_max=None, slow_radius=0.5):
        """(v, w) towards a target point; reverse drives backwards."""
        x, y, yaw = pose
        v_max = self.v_max if v_max is None else v_max
        heading = yaw + (math.pi if reverse else 0.0)

        dx = target[0] - x
        dy = target[1] - y
        dist = math.hypot(dx, dy)

        heading_error = wrap(math.atan2(dy, dx) - heading)  # 0 if in front, + if on the left, - if on the right

        if abs(heading_error) > self.sharp_turn_above: # slowly forward or backward while turning without turning in place
            w = max(-self.w_max, min(self.w_max, 1.5 * heading_error))
            vc = min(self.sharp_turn_speed, 0.5 * v_max)
            return (-vc if reverse else vc), w

        # heading + distance factors if turn angle is high, decrease speed; if distance is high, max speed otherwise min speed. (+0.3)
        v = v_max * max(0.2, math.cos(heading_error)) * min(1.0, dist / max(slow_radius, 1e-3) + 0.3)
        w = 2.0 * v * math.sin(heading_error) / max(dist, self.lookahead * 0.5)
        w = max(-self.w_max, min(self.w_max, w))
        return (-v if reverse else v), w
