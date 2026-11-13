"""
Contains mission logic. It is ROS independent and its methods are called directly from ROS mission node.
"""
import math
from collections import deque
from collections.abc import Callable
from . import actions as A
from .dem import OrbitalDem
from .geometry import Circle, Point, Pose, point_segment_distance
from .navigation import CostMap, PurePursuit
from .dig_area import select_area
from .site import Hopper, SiteModel, antenna_stage
from .strategies import STRATEGIES, CycleEstimator, charge_decision

def hopper_from_params(p):
    return Hopper(float(p["hopper_x"]), float(p["hopper_y"]), math.radians(p["hopper_yaw_deg"]), float(p["hopper_length"]),
                  float(p["hopper_width"]), wall=float(p["hopper_wall"]), height=float(p["hopper_height"]))


TOTAL_COUNTERS = ("delivered_kg", "dumped_kg", "dug_kg", "consumed_wh", "charged_wh")


class Mission:
    def __init__(self, params: dict, dem_info: dict, clock: Callable[[], float], log: Callable[[str], None] = print, publish_path: Callable[[list[Point]], None] | None = None) -> None:
        self.p: dict = dict(params)
        if self.p["strategy"] not in STRATEGIES:
            raise ValueError(f"unknown strategy '{self.p['strategy']}': use one of {', '.join(STRATEGIES)}")
        
        self.clock: Callable[[], float] = clock
        self.log: Callable[[str], None] = log
        self.publish_path: Callable[[list[Point]], None] = publish_path or (lambda pts: None)  # for RViz
        p = self.p

        hopper = hopper_from_params(p)
        self.site: SiteModel = SiteModel(hopper=hopper, max_passes=p["max_passes"], antenna_xy=(p["antenna_x"], p["antenna_y"]))
        self.costmap: CostMap = CostMap(half_size=25.0, resolution=0.2, inflation=p["inflation"])
        self.pp: PurePursuit = PurePursuit(p["lookahead"], p["v_max"], p["w_max"])
        self.est: CycleEstimator = CycleEstimator(p, self.site)

        self.pose: Pose | None = None
        self.state: dict = {}                       # merged JSON of /rover/battery, /rover/excavator, /isru/hopper
        self.dem: OrbitalDem = OrbitalDem(dem_info) # dig area and steep cells of the cost map
        self.obstacles: list[Circle] = []           # known rocks in the cost map
        self.site_known: bool = False               # /obstacles received
        self.lander: Circle | None = None
        self.infrastructure: list[Circle] = []      # lander, hopper...
        self.queue: deque[A.Action] = deque()
        self.current: A.Action | None = None
        self.cmd_v: float = 0.0                     # m/s
        self.cmd_w: float = 0.0                     # rad/s
        self.impl: dict[str, float] = {"arm_front": p["transport_arm_deg"], "arm_rear": p["transport_arm_deg"], "drum_front": 0.0, "drum_rear": 0.0}
        self.t_start: float | None = None
        self.consec_failures: int = 0
        self.t_last_step: float | None = None
        self.t_end: float | None = None
        self.state0: dict[str, float] = {}          # TOTAL_COUNTERS at mission start
        self.avg_power: float = 60.0                # first guess (W), replaced by the actual average
        self.pw_ref: tuple[float, float] | None = None
        self.finished: bool = False
        self.end_reason: str = ""
        self.waiting_logged: bool = False

    def now(self):
        """Return current time."""
        return self.clock()

    def mission_time(self):
        """Returns mission time. Total time if mission has ended."""
        if self.t_start is None:
            return 0.0
        return (self.t_end if self.t_end is not None else self.now()) - self.t_start

    def cmd(self, v:float, w:float):
        """Set linear speed and angular speed commands."""
        self.cmd_v = float(v)
        self.cmd_w = float(w)

    def stop_base(self):
        """Stop the rover."""
        self.cmd_v = self.cmd_w = 0.0

    def set_implements(self, arm_front=None, arm_rear=None, drum_front=None, drum_rear=None):
        """Set arms and drums angles/speeds."""
        for k, v in (("arm_front", arm_front), ("arm_rear", arm_rear), ("drum_front", drum_front), ("drum_rear", drum_rear)):
            if v is not None:
                self.impl[k] = float(v)

    def event(self, name, **details):
        """Prints one console line in the terminal."""
        t = round(self.mission_time(), 2)
        self.log(f"[{t:7.1f}s] {name} {details}")

    def set_pose(self, x, y, yaw):
        """Update rover pose."""
        self.pose = (x, y, yaw)

    def set_state(self, state):
        """Update mission state."""
        self.state = state
        if not state.get("charging", False):
            t = self.now()
            e = state.get("consumed_wh", 0.0)
            if self.pw_ref is None:
                self.pw_ref = (t, e)
            elif t - self.pw_ref[0] >= 5.0:
                pw = (e - self.pw_ref[1]) * 3600.0 / (t - self.pw_ref[0])
                if 0.0 < pw < 2000.0:
                    self.avg_power = 0.9 * self.avg_power + 0.1 * pw
                self.pw_ref = (t, e)
        else:
            self.pw_ref = None

    def set_obstacles(self, rocks: list[Circle], lander: Circle | None, infrastructure: list[Circle]) -> None:
        """Update obstacles."""
        self.obstacles = list(rocks)
        self.lander = lander
        self.infrastructure = list(infrastructure)
        self.site_known = True

    def build_costmap(self) -> None:
        circles = self.obstacles + self.infrastructure + ([self.lander] if self.lander is not None else [])
        for x, y, radius in circles:
            self.costmap.add_circle(x, y, radius)
        self.costmap.add_mask(self.dem.steep_mask(self.costmap.X, self.costmap.Y, self.p["max_drive_slope_deg"]))

    def corridor_clear(self, a, b, half_width=0.4):
        """No known obstacle within half_width of the segment ab."""
        return all(point_segment_distance((o[0], o[1]), a, b) > o[2] + half_width for o in self.obstacles)

    def ready(self):
        """All inputs present. Logs once what it is still waiting for."""
        missing = [k for k, ok in (("pose", self.pose is not None), ("state", bool(self.state)), ("obstacles", self.site_known)) if not ok]
        if missing and not self.waiting_logged and self.pose is not None:
            self.waiting_logged = True
            self.log("Waiting for: " + ", ".join(missing))
        return not missing

    def choose_area(self):
        """Choose best area to dig respecting rules."""
        if self.site.area is not None:
            return True
        
        obstacles = list(self.obstacles) + list(self.infrastructure)
        area = select_area(self.p, self.dem, obstacles, self.lander, self.site.hopper, self.site.antenna, self.est)
        if area is None:
            self.finish("no feasible dig area")
            return False
        
        self.site.set_area(area)
        self.site.check_lane_obstacles(self.obstacles)
        self.event("dig_area", x=round(area.cx, 2), y=round(area.cy, 2), corners=[[round(v, 2) for v in c] for c in area.corners()])
        return True

    def plan_next(self):
        """Plan what to do next: haul, dig, recharge."""
        p = self.p
        xy = self.pose[:2]
        st = self.state

        # if more than half of the capacity is gone, go unload
        if st.get("load_kg", 0.0) >= 0.5 * st.get("capacity_kg", 2.0 * p["drum_capacity_kg"]):
            self.queue_haul()
            return True

        lane = self.site.next_lane() # find next lane
        time_left = p["max_mission_time"] - self.mission_time()
        target = charge_decision(p["strategy"], st, xy, lane, self.est, p, self.avg_power, time_left)
        if target is not None and st.get("soc", 1.0) < target - 0.01:
            self.queue_charge(target)
            return True
        if lane is None:
            return False
        self.queue_cycle(lane)
        return True

    def queue_charge(self, target_soc):
        """Drive to the charging antenna and recharge until SoC reaches target_soc."""
        p = self.p
        antenna = (p["antenna_x"], p["antenna_y"])
        stage = antenna_stage(p)
        self.queue.extend([
            A.Implements(p["transport_arm_deg"], p["transport_arm_deg"], 0.0, 0.0, hold=0.5),
            A.NavigateTo(stage[0], stage[1], yaw=math.radians(p["antenna_yaw_deg"]), label="antenna stage"),
            A.StraightTo(antenna[0], antenna[1], speed=p["approach_speed"], line_from=stage),
            A.Charge(target_soc),
            A.StraightTo(stage[0], stage[1], reverse=True),
        ])

    def queue_cycle(self, lane):
        """Drive to the lane, dig along it until both drums."""
        p = self.p
        a = lane.heading
        sx, sy = self.est.lane_start(lane)
        self.queue.extend([
            A.Implements(p["transport_arm_deg"], p["transport_arm_deg"], 0.0, 0.0, hold=0.5),
            A.NavigateTo(sx, sy, yaw=a, label=f"strip {lane.index} (pass {lane.passes + 1})"),
            A.DigLane(lane, p["dig_arm_deg"], p["transport_arm_deg"], p["dig_speed"], p["drum_offset"]),
        ])
        self.queue_haul()

    def queue_haul(self):
        """Drive to the hopper, approach, move arms and unload. Reverse, rotate and repeat."""
        p = self.p
        hopper = self.site.hopper
        stage = hopper.stage(p["hopper_stage_dist"])
        dump = hopper.dump_point(p["hopper_standoff"])
        turn = hopper.stage(hopper.length / 2 + hopper.wall + p["hopper_standoff"] + p["hopper_turn_dist"])
        dump_arm = p["dump_arm_deg"]
        carry = p["transport_arm_deg"]
        move_s = A.arm_travel_time(carry, dump_arm) + 0.5
        acts = [
            A.Implements(carry, carry, 0.0, 0.0, hold=0.5),
            A.NavigateTo(stage[0], stage[1], yaw=hopper.yaw, label="hopper stage"),
            A.StraightTo(dump[0], dump[1], speed=p["approach_speed"], tol=0.03, line_from=stage, timeout=90.0),
            A.Implements(dump_arm, carry, 0.0, 0.0, hold=move_s),
            A.UnloadAtHopper("front"),
            A.Implements(carry, carry, 0.0, 0.0, hold=move_s),
            A.StraightTo(turn[0], turn[1], reverse=True, line_from=dump),
            A.RotateTo(hopper.yaw + math.pi, creep=0.0),
            A.StraightTo(dump[0], dump[1], reverse=True, speed=p["approach_speed"], tol=0.03, line_from=turn, timeout=90.0),
            A.Implements(carry, dump_arm, 0.0, 0.0, hold=move_s),
            A.UnloadAtHopper("rear"),
            A.Implements(carry, carry, 0.0, 0.0, hold=move_s),
            A.StraightTo(stage[0], stage[1], line_from=dump, timeout=90.0),
        ]
        for act in acts:
            act.haul = True
        acts[0].haul_start = True
        self.queue.extend(acts)

    def on_failure(self, action):
        """In case of failure of action executes fallback actions."""
        self.consec_failures += 1
        self.event("action_failed", action=action.describe(), reason=action.reason, consecutive=self.consec_failures)
        self.queue.clear()
        p = self.p
        x, y, yaw = self.pose
        # back off 0.5 m in reverse before planning again
        back = (x - 0.5 * math.cos(yaw), y - 0.5 * math.sin(yaw))
        self.queue.extend([
            A.Implements(p["transport_arm_deg"], p["transport_arm_deg"], 0.0, 0.0, hold=1.0),
            A.StraightTo(back[0], back[1], reverse=True, timeout=15.0),
        ])
        if self.consec_failures >= 8:
            self.finish("too many consecutive failures")

    def step(self):
        """A tick in the mission control. When invoked by ROS timers it chooses what to do depending on current state."""
        if self.finished:
            self.stop_base()
            self.set_implements(drum_front=0.0, drum_rear=0.0)
            return
        if not self.ready():
            self.stop_base()
            return
        
        now = self.now()
        if self.t_start is None: # first step execution
            self.t_start = now
            self.state0 = {k: self.state.get(k, 0.0) for k in TOTAL_COUNTERS}
            self.build_costmap()
            if not self.choose_area():
                return
            self.event("mission_start", strategy=self.p["strategy"])
            h = self.site.hopper
            p = self.p
            for name, a, b in (("hopper", h.stage(p["hopper_stage_dist"]), h.center), ("antenna", antenna_stage(p), (p["antenna_x"], p["antenna_y"]))):
                if not self.corridor_clear(a, b):
                    self.event("approach_blocked", target=name, hint="an obstacle is blocking the approach to the hopper and/or the recharge antenna")
        
        if self.t_last_step is not None and now < self.t_last_step - 1.0: # if for any reason unity execution got interrupted/restarted
            self.t_end = self.t_last_step
            self.finish("simulation restarted")
            return

        self.t_last_step = now
        tgt = self.p["target_delivered_kg"]
        if tgt > 0 and self.get_total_counter("delivered_kg") >= tgt:
            self.finish("delivery target reached")
            return
        
        if self.mission_time() >= self.p["max_mission_time"]:
            self.finish("time limit exceeded")
            return

        if self.current is None:
            if not self.queue and not self.plan_next():
                self.finish("finished digging selected area")
                return
            self.current = self.queue.popleft()
            
            if getattr(self.current, "haul_start", False) and self.state.get("load_kg", 0.0) < self.p["min_haul_kg"]:
                while self.queue and self.queue[0].haul: # dug less than required to make a unload trip worth it
                    self.queue.popleft()
                self.current = None
                return
            self.current.start(self)

        status = self.current.update(self)
        if status == A.DONE:
            if isinstance(self.current, (A.DigLane, A.Charge, A.UnloadAtHopper)):
                self.consec_failures = 0
            self.current = None
        elif status == A.FAILED:
            failed =  self.current
            self.current = None
            self.stop_base()
            self.on_failure(failed)

    def get_total_counter(self, key):
        """A cumulative Unity counter, counted from the start of this mission."""
        return self.state.get(key, 0.0) - self.state0.get(key, 0.0)

    def finish(self, reason):
        """Terminates the mission with a reason."""
        if self.finished:
            return
        self.finished = True
        self.stop_base()
        self.set_implements(drum_front=0.0, drum_rear=0.0)
        if self.t_end is None:
            self.t_end = self.now()
        self.end_reason = reason
        self.event("mission_end", reason=reason, delivered_kg=round(self.get_total_counter("delivered_kg"), 1))
