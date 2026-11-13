"""
Contains info regarding an area for digging.
"""
import math
from dataclasses import dataclass, field
import numpy as np
from .geometry import point_segment_distance
from .site import Lane, antenna_stage


@dataclass
class DigArea:
    cx: float                                        # area center x coordinate
    cy: float                                        # area center y coordinate
    yaw: float                                       # digging direction
    length: float                                    # area length in meters
    width: float                                     # area width in meters
    strips: list[Lane] = field(default_factory=list) # area digging strips
    cycle_duration_s: float = 0.0                    # expected cycle time

    def to_local(self, x, y):
        """Convert scene coordinates to local ones relative to this area center."""
        dx = x - self.cx
        dy = y - self.cy
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return dx * c + dy * s, -dx * s + dy * c

    def to_map(self, a, b):
        """Convert local coordinates to global ones."""
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return self.cx + a * c - b * s, self.cy + a * s + b * c

    def corners(self):
        """Return this area four corners."""
        half_len = self.length / 2
        half_wid = self.width / 2
        return [self.to_map(a, b) for a, b in ((-half_len, -half_wid), (half_len, -half_wid), (half_len, half_wid), (-half_len, half_wid))]


def make_area(cx, cy, yaw, parameters):
    """Create DigArea given its center, orientation."""
    area = DigArea(cx, cy, yaw, parameters["area_length"], parameters["area_width"])
    n = max(1, int(round(area.width / parameters["lane_spacing"])))
    sp = area.width / n
    for i in range(n):
        b = -area.width / 2 + sp * (i + 0.5)
        x0, y0 = area.to_map(-area.length / 2, b)
        area.strips.append(Lane(i, x0, y0, math.degrees(yaw), area.length))
    return area


def sample_points(area:DigArea, step:float =0.5, lead_in:float=1.0):
    """Points covering the area and the rover way in before the lane starts."""
    a_vals = np.arange(-area.length / 2, area.length / 2 + 1e-6, step)
    b_vals = np.arange(-area.width / 2, area.width / 2 + 1e-6, step)
    inside = [area.to_map(a, b) for a in a_vals for b in b_vals]
    lead = [area.to_map(-area.length / 2 - lead_in, b) for b in b_vals]
    return inside, lead


def evaluate(area: DigArea, p, dem, obstacles, lander, hopper, antenna, est):
    """Fills the area's features returns the reason it is unfeasable, or None."""
    inside, lead = sample_points(area)
    allpts = inside + lead
    if any(math.hypot(x, y) > p["area_r_max"] for x, y in allpts):
        return "too far"
    
    clearance = p["area_clearance"]
    if lander is not None and any(math.hypot(x - lander[0], y - lander[1]) < lander[2] + clearance for x, y in allpts):
        return "lander"
    
    hopper_stage = hopper.stage(p["hopper_stage_dist"] + 0.5)
    ds = antenna_stage(p, extra=0.5)
    for a, b in ((hopper_stage, hopper.center), (ds, antenna)):
        if any(point_segment_distance(q, a, b) < clearance for q in allpts):
            return "hopper approach"
        
    for ox, oy, r in obstacles:
        u, v = area.to_local(ox, oy)
        if abs(u) < area.length / 2 + r + 0.3 and abs(v) < area.width / 2 + r + 0.3:
            return "obstacle"
        
    if any(dem.cell(x, y) is None for x, y in inside):
        return "outside the DEM"

    # estimate cycle duration
    stage = est.hopper_stage()
    load = 2.0 * p["drum_capacity_kg"]
    times = []
    for lane in area.strips:
        advance = min(load / est.yield_kg_m, lane.length)
        for s in (area.length / 6, area.length / 2, 5 * area.length / 6):
            times.append(
                         est.travel_time(stage, lane.point(max(s - advance / 2 - p["drum_offset"], -1.0)))
                         + est.travel_time(lane.point(min(s + advance / 2, lane.length) + p["drum_offset"]), stage)
                         + load / est.yield_kg_m / p["dig_speed"]
                         )
    area.cycle_duration_s = float(np.mean(times)) + est.unload_time()
    return None


def select_area(p, dem, obstacles, lander, hopper, antenna, est):
    """Selects the best diggable area."""
    stage = hopper.stage(p["hopper_stage_dist"])
    feasible = []
    grid = p["area_grid"]
    radius = p["area_r_max"]
    
    for cx in np.arange(-radius, radius + 1e-6, grid):
        for cy in np.arange(-radius, radius + 1e-6, grid):
            if math.hypot(cx, cy) > radius:
                continue
            yaw = math.atan2(cy - stage[1], cx - stage[0])
            area = make_area(float(cx), float(cy), yaw, p)
            if evaluate(area, p, dem, obstacles, lander, hopper, antenna, est) is None:
                feasible.append(area)
    if not feasible:
        return None
    return min(feasible, key=lambda area: (round(area.cycle_duration_s, 3), area.cx, area.cy))
