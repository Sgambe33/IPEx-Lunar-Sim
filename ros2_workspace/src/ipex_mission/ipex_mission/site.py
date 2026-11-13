"""
Lunar site model that contains the dig area, the hopper and the charging antenna.
"""
import json
import math
import os
from dataclasses import dataclass
from typing import TYPE_CHECKING
from ament_index_python.packages import get_package_share_directory
from .geometry import Point, point_segment_distance

if TYPE_CHECKING:
    from .dig_area import DigArea


@dataclass
class Lane:
    """A straight diggable lane from (x0, y0) along heading_deg for length metres."""
    index: int             # lane index
    x0: float              # lane x origin coordinate
    y0: float              # lane y origin coordinate
    heading_deg: float     # lane heading in degrees
    length: float          # lane legth in meters
    next_s: float = 0.0    # distance along the lane where untouched regolith starts
    passes: int = 0        # how many times rover wen over the lane
    blocked: bool = False  # a known rock in the lane corridor
    failures: int = 0

    @property
    def heading(self):
        return math.radians(self.heading_deg)

    def point(self, s):
        """Map point s metres along the lane."""
        return self.x0 + s * math.cos(self.heading), self.y0 + s * math.sin(self.heading)

    def along(self, x, y):
        """Distance of (x, y) along the strip from its start."""
        return (x - self.x0) * math.cos(self.heading) + (y - self.y0) * math.sin(self.heading)

    def end_pass(self):
        self.passes += 1
        self.next_s = 0.0


@dataclass
class Hopper:
    """Hopper where regolith is unloaded."""
    x: float               # hopper x coordinate
    y: float               # hopper y coordinate
    yaw: float             # hopper heading
    length: float = 0.5    # hopper sizes
    width: float = 0.7
    wall: float = 0.03
    height: float = 0.28

    @property
    def center(self):
        return self.x, self.y

    def stage(self, dist):
        """The stage is a point dist metres before the hopper center on the approach line."""
        return self.x - dist * math.cos(self.yaw), self.y - dist * math.sin(self.yaw)

    def dump_point(self, standoff):
        """Returns the point where the rover stops to unload, standoff metres before the wall."""
        return self.stage(self.length / 2 + self.wall + standoff)

    def contains(self, px, py, margin=0.0):
        """Checks if a point is inside the hopper limits."""
        dx = px - self.x
        dy = py - self.y
        a = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        b = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        return abs(a) <= self.length / 2 + margin and abs(b) <= self.width / 2 + margin


class SiteModel:
    def __init__(self, hopper: Hopper, max_passes: int, antenna_xy: Point) -> None:
        self.max_passes: int = max_passes
        self.hopper: Hopper = hopper
        self.antenna: Point = antenna_xy
        self.area: DigArea | None = None
        self.lanes: list[Lane] = []

    def set_area(self, area):
        self.area = area
        self.lanes = list(area.strips)

    def check_lane_obstacles(self, obstacles, clearance=0.45):
        """Marks lanes blocked if an obstacle is too close."""
        for lane in self.lanes:
            a = lane.point(-1.0)
            b = lane.point(lane.length + 1.0)
            lane.blocked = any(point_segment_distance((ox, oy), a, b) < o_rad + clearance for ox, oy, o_rad in obstacles)

    def usable_lanes(self):
        """Lanes that can still be dug not blocked, passes left, not failing."""
        return [lane for lane in self.lanes if not lane.blocked and lane.passes < self.max_passes and lane.failures < 3]

    def next_lane(self):
        """Returns next lane to dig based on remaining ones."""
        lanes = self.usable_lanes()
        if not lanes:
            return None
        return min(lanes, key=lambda ln: (ln.passes, ln.index))


def antenna_stage(parameters:dict, extra=0.0):
    """Start of the straight final approach to the antenna."""
    a = math.radians(parameters["antenna_yaw_deg"])
    hx = math.cos(a)
    hy = math.sin(a)
    d = parameters["antenna_stage_dist"] + extra
    return parameters["antenna_x"] - d * hx, parameters["antenna_y"] - d * hy

def config_path(name):
    """Returns path config/<name> of the installed package."""
    path = os.path.join(get_package_share_directory("ipex_mission"), "config", name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} missing: export it from Unity into config/ (see README) and run colcon build")
    return path

def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)

def apply_site_survey(parameters:dict, survey):
    """Populate infrastructure parameters by parsing survey file."""
    h = survey["hopper"]
    parameters.update({"hopper_x": h["x"], "hopper_y": h["y"], "hopper_yaw_deg": math.degrees(h["yaw"]), "hopper_length": h["length"], 
              "hopper_width": h["width"], "hopper_wall": h["wall"], "hopper_height": h["height"]})
    charger = survey["charger"]
    lander = survey["lander"]
    ux = charger["x"] - lander["x"]
    uy = charger["y"] - lander["y"]
    n = math.hypot(ux, uy)
    ux, uy = (ux / n, uy / n) if n > 1e-6 else (-1.0, 0.0)
    side = {"left": 1.0, "right": -1.0}[parameters["antenna_charger_side"]]
    standoff = parameters["antenna_side_gap"] + parameters["rover_half_width"]
    parameters.update({"antenna_x": charger["x"] + standoff * ux, "antenna_y": charger["y"] + standoff * uy, "antenna_yaw_deg": math.degrees(math.atan2(side * ux, -side * uy)), "charging_radius": charger["radius"]})
    return parameters

