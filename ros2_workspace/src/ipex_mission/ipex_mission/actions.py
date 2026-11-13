"""
Primitive actions executed by the mission node in sequences.
"""
from __future__ import annotations
import math
from typing import TYPE_CHECKING
from .geometry import Point, Pose, wrap
from .site import Lane

if TYPE_CHECKING:
    from .mission_core import Mission

RUNNING = "running"
DONE = "done"
FAILED = "failed"
ARM_RATE_DEG_S = 17.25


def arm_travel_time(from_deg, to_deg):
    """Seconds an arm needs to move between two angles."""
    return abs(to_deg - from_deg) / ARM_RATE_DEG_S


class Action:
    """Base class for all actions. Declares common methods overridden by subclasses."""
    name = "action"
    timeout = 300.0

    def __init__(self) -> None:
        self.t0: float | None = None    # action's start time
        self.reason: str = ""           # action's fail reason
        self.haul: bool = False         # true if this action is part of a sequence of actions that hauls regolith to the hopper

    def start(self, ctx:Mission):
        """Starts the action."""
        self.t0 = ctx.now()

    def elapsed(self, ctx:Mission):
        """Returns the time passed since the action started."""
        return ctx.now() - self.t0

    def update(self, ctx:Mission):
        """Used to monitor action execution and act accordingly."""
        return DONE

    def fail(self, reason:str):
        """Invoked to flag an action as failed specifying the reason."""
        self.reason = reason
        return FAILED

    def describe(self):
        return self.name


class StuckMonitor:
    """Checks whether the rover has moved by min_motion or turned by min_turn_deg in the last window seconds."""

    def __init__(self, window: float = 10.0, min_motion: float = 0.05, min_turn_deg: float = 10.0) -> None:
        self.window: float = window                         # s
        self.min_motion: float = min_motion                 # m
        self.min_turn: float = math.radians(min_turn_deg)   
        self.ref: Pose | None = None                        # stored reference pose 
        self.t_ref: float | None = None                     # when ref pose was stored

    def check(self, ctx:Mission, is_commanding_motion:bool):
        x, y, yaw = ctx.pose
        t = ctx.now()
        if not is_commanding_motion or self.ref is None:
            self.ref = (x, y, yaw)
            self.t_ref = t
            return False
        if (math.hypot(x - self.ref[0], y - self.ref[1]) > self.min_motion or abs(wrap(yaw - self.ref[2])) > self.min_turn):
            self.ref = (x, y, yaw)
            self.t_ref = t
            return False
        return t - self.t_ref > self.window


class Implements(Action):
    """Action to set arm angles and drum commands, then waits hold seconds (arms move at ARM_RATE_DEG_S)."""
    name = "implements"

    def __init__(self, arm_front: float | None = None, arm_rear: float | None = None, drum_front: float | None = None, drum_rear: float | None = None, hold: float = 1.5) -> None:
        super().__init__()
        self.args = (arm_front, arm_rear, drum_front, drum_rear)
        self.hold: float= hold

    def start(self, ctx):
        super().start(ctx)
        ctx.set_implements(*self.args)
        ctx.stop_base()

    def update(self, ctx):
        ctx.stop_base()
        return DONE if self.elapsed(ctx) >= self.hold else RUNNING


class RotateTo(Action):
    """Turns to a specified heading. Move forward a little during wide turns so the rover does not turn in place."""
    name = "rotate"
    timeout = 60.0
    MIN_W = 0.15        # rad/s
    BOOST_RATE = 0.1    # rad/s added per second without progress
    PROGRESS_DEG = 0.5  # rotation that counts as progress

    def __init__(self, yaw: float, tol_deg: float = 5.0, creep: float = 0.04, creep_until_deg: float = 20.0) -> None:
        super().__init__()
        self.yaw: float = yaw                           # heading to rotate to in rad
        self.tol: float = math.radians(tol_deg)         # allowed tolerance in final heading
        self.creep: float = creep                       # speed used to rotate to requested heading in m/s
        self.creep_until: float = math.radians(creep_until_deg) # when creep_until_deg are left in the turn, the turn is made in place
        self.stuck: StuckMonitor = StuckMonitor(window=15.0, min_motion=0.03, min_turn_deg=5.0)
        self.boost: float = 0.0
        self.best_err: float | None = None              # best heading error reached yet
        self.t_best: float | None = None                # when best_err was achieved

    def update(self, ctx):
        heading_error = wrap(self.yaw - ctx.pose[2])
        if abs(heading_error) < self.tol:
            ctx.stop_base()
            return DONE
        if self.elapsed(ctx) > self.timeout:
            return self.fail("rotate timeout")
        now = ctx.now()
        # if rover is actually turning then has made progress
        if self.best_err is None or abs(heading_error) < self.best_err - math.radians(self.PROGRESS_DEG):
            self.best_err = abs(heading_error)
            self.t_best = now
            self.boost = 0.0
        elif now - self.t_best > 1.0: # no progress has been made in the last second, help the rover by increasing speed
            self.boost = min(ctx.pp.w_max, self.BOOST_RATE * (now - self.t_best - 1.0))
        
        w = max(-ctx.pp.w_max, min(ctx.pp.w_max, 1.5 * heading_error))
        min_allowed = min(ctx.pp.w_max, self.MIN_W + self.boost)
        if abs(w) < min_allowed:
            w = math.copysign(min_allowed, heading_error)

        ctx.cmd(self.creep if abs(heading_error) > self.creep_until else 0.0, w)
        if self.stuck.check(ctx, True):
            return self.fail("rotate stuck")
        return RUNNING


class StraightTo(Action):
    """Drives to a point along a straight line without obstacle checks."""
    MAX_APPROACH_ANGLE = math.radians(15.0) # max angle to approach the line
    APPROACH_ANGLE_PER_M = 2.0              # how many rads for each offset meter
    HEADING_GAIN = 1.5                      # angular speed for each radiant of error

    name = "straight"

    def __init__(self, x: float, y: float, reverse: bool = False, speed: float = 0.12, tol: float = 0.06, timeout: float = 60.0, line_from: Point | None = None) -> None:
        super().__init__()
        self.goal: Point = (x, y)
        self.line_from: Point | None = line_from     # start of the line to follow; None = where the rover starts
        self.reverse: bool = reverse
        self.speed: float = speed                    # m/s
        self.tol: float = tol                        # m
        self.timeout: float = timeout                # s
        self.stuck: StuckMonitor = StuckMonitor(window=8.0)

    def start(self, ctx):
        super().start(ctx)
        if self.line_from is None:
            self.line_from = ctx.pose[:2]

    def update(self, ctx):
        x, y, yaw = ctx.pose
        ax, ay = self.line_from
        gx, gy = self.goal
        dist = max(math.hypot(gx - ax, gy - ay), 1e-6)
        ux = (gx - ax) / dist # get travel direction
        uy = (gy - ay) / dist
        distance_to_go = (gx - x) * ux + (gy - y) * uy
        current_lateral_offset = -(x - ax) * uy + (y - ay) * ux
        
        if distance_to_go < self.tol:
            ctx.stop_base()
            return DONE
        if self.elapsed(ctx) > self.timeout:
            return self.fail("straight timeout")
        
        line_yaw = math.atan2(uy, ux)
        sign = -1.0 if self.reverse else 1.0
        approach_angle = max(-self.MAX_APPROACH_ANGLE, min(self.MAX_APPROACH_ANGLE, self.APPROACH_ANGLE_PER_M * current_lateral_offset))
        yaw_needed = wrap(line_yaw + (math.pi if self.reverse else 0.0) - approach_angle)
        yaw_error = wrap(yaw_needed - yaw)
        w = max(-ctx.pp.w_max, min(ctx.pp.w_max, self.HEADING_GAIN * yaw_error))

        if abs(yaw_error) > math.radians(30.0):
            v = 0.0
        else:
            v = self.speed * max(0.3, min(1.0, distance_to_go / 0.3)) * math.cos(yaw_error)
        ctx.cmd(sign * v, w)

        if self.stuck.check(ctx, abs(v) > 0.02):
            return self.fail("stuck")
        return RUNNING

    def describe(self):
        return f"straight({self.goal[0]:.1f},{self.goal[1]:.1f}{',rev' if self.reverse else ''})"


class NavigateTo(Action):
    """Executes A* on the cost map, then uses pure pursuit to follow the path, then optional final heading alignment."""
    name = "navigate"

    def __init__(self, x: float, y: float, yaw: float | None = None, tol: float = 0.15, timeout: float = 400.0, label: str = "") -> None:
        super().__init__()
        self.goal: Point = (x, y)
        self.yaw: float | None = yaw                 # final heading (rad), None = any
        self.tol: float = tol                        # m
        self.timeout: float = timeout                # s
        self.label: str = label
        self.path: list[Point] | None = None
        self.t_plan: float = -1e9
        self.rotate: RotateTo | None = None
        self.stuck: StuckMonitor = StuckMonitor(window=10.0)
        self.replans: int = 0

    def plan(self, ctx:Mission):
        self.path = ctx.costmap.plan(ctx.pose[:2], self.goal) # from pose to goal
        self.t_plan = ctx.now()
        self.replans += 1
        if self.path:
            ctx.publish_path(self.path)

    def start(self, ctx):
        super().start(ctx)
        self.plan(ctx)

    def update(self, ctx):
        if self.elapsed(ctx) > self.timeout:
            return self.fail("navigate timeout")
        if self.rotate is not None:
            st = self.rotate.update(ctx)
            return st if st != FAILED else self.fail(self.rotate.reason)

        x, y, _ = ctx.pose
        goal_distance = math.hypot(self.goal[0] - x, self.goal[1] - y)
        end = self.path[-1] if self.path else self.goal
        end_distance = math.hypot(end[0] - x, end[1] - y)

        # if close enough and/or heading is correct stop and complete
        if goal_distance < self.tol or (end_distance < self.tol and goal_distance < 1.0):
            ctx.stop_base()
            if self.yaw is None:
                return DONE
            self.rotate = RotateTo(self.yaw)
            self.rotate.start(ctx)
            return RUNNING
        if end_distance < self.tol:
            ctx.stop_base()
            return self.fail("goal unreachable")

        if self.path is None:
            if ctx.now() - self.t_plan > 2.0:
                self.plan(ctx)
            if self.path is None:
                ctx.stop_base()
                return self.fail("no path") if self.replans > 5 else RUNNING

        target, current_lateral_offset = ctx.pp.target_point(ctx.pose, self.path)
        if current_lateral_offset > 0.8 or ctx.now() - self.t_plan > 15.0:
            self.plan(ctx)
            if self.path is None:
                return RUNNING
            target, _ = ctx.pp.target_point(ctx.pose, self.path)

        if end_distance < ctx.pp.lookahead:
            target = end
        v, w = ctx.pp.command(ctx.pose, target)
        ctx.cmd(v, w)
        if self.stuck.check(ctx, abs(v) > 0.02 or abs(w) > 0.05):
            return self.fail("stuck")
        return RUNNING

    def describe(self):
        return f"navigate({self.label or f'{self.goal[0]:.1f},{self.goal[1]:.1f}'})"


class DigLane(Action):
    """
    Drives and digs along a lane with one drum at a time. Composed of four phases:
    - lowering: when one of the two arms is moving either up or down
    - front: when front drum is digging
    - travel: when travelling along the lane
    - rear: when rear drum is digging
    """
    name = "dig"

    def __init__(self, lane: Lane, dig_arm_deg: float, transport_arm_deg: float, dig_speed: float, drum_offset: float, fill_fraction: float = 0.95, timeout: float = 300.0) -> None:
        super().__init__()
        self.lane: Lane = lane                          # lane to dig
        self.dig_arm: float = dig_arm_deg               # elevation of arm to dig
        self.transport_arm: float = transport_arm_deg   # elevation of arm when movng
        self.dig_speed: float = dig_speed               # lane digging speed
        self.travel_speed: float = 2.0 * dig_speed      # between the front arm and rear arm digging
        self.drum_offset: float = drum_offset           # distance from rover center to drum
        self.fill: float = fill_fraction                # how much to fill each drum in percentage, 100%=15kg circa
        self.timeout: float = timeout                   # max time to execute action
        self.phase: str = "lowering"                    # lowering | front | travel | rear
        self.t_phase: float = 0.0
        self.front_end_s: float | None = None
        self.stuck: StuckMonitor = StuckMonitor(window=12.0, min_motion=0.03)

    def start(self, ctx):
        super().start(ctx)
        ctx.set_implements(self.dig_arm, self.transport_arm, 1.0, 0.0) # lower front arm
        self.t_phase = ctx.now()

    def distance_along(self, ctx):
        """Distance of the rover centre from the start of current lane."""
        x, y, _ = ctx.pose
        return self.lane.along(x, y)

    def finish(self, ctx):
        """Records the dig on the lane, raises the arms and stops. Returns DONE."""
        s = self.distance_along(ctx)
        rear_s = s - self.drum_offset
        if self.phase == "rear":
            self.lane.next_s = max(self.lane.next_s, rear_s)
        elif self.front_end_s is not None:
            self.lane.next_s = max(self.lane.next_s, self.front_end_s)
        else:
            self.lane.next_s = max(self.lane.next_s, s + self.drum_offset)
        if self.lane.next_s + 2.0 * self.drum_offset + 1.2 > self.lane.length:
            self.lane.end_pass()
        ctx.set_implements(self.transport_arm, self.transport_arm, 0.0, 0.0)
        ctx.stop_base()
        return DONE

    def update(self, ctx):
        st = ctx.state
        capacity = st.get("capacity_kg", 30.0) / 2.0
        s = self.distance_along(ctx)
        if self.phase == "lowering": # wait until the front arm has been lowered
            ctx.stop_base()
            if ctx.now() - self.t_phase > arm_travel_time(self.transport_arm, self.dig_arm) + 0.5:
                self.phase = "front"
            return RUNNING

        if self.phase == "front" and st.get("front_load_kg", 0.0) >= self.fill * capacity:
            self.front_end_s = s + self.drum_offset
            ctx.set_implements(self.transport_arm, None, 0.0, None)
            self.phase = "travel"
            
        if self.phase == "travel":
            lead = arm_travel_time(self.transport_arm, self.dig_arm) * self.travel_speed
            if s - self.drum_offset >= self.front_end_s - 0.05 - lead:
                ctx.set_implements(None, self.dig_arm, None, 1.0)
                self.phase = "rear"

        rear_full = self.phase == "rear" and st.get("rear_load_kg", 0.0) >= self.fill * capacity
        lane_end = s + self.drum_offset > self.lane.length + 0.3
        if rear_full or lane_end or self.elapsed(ctx) > self.timeout:
            return self.finish(ctx)

        # follow the lane
        lookahead = self.lane.point(s + 0.8)
        v_max = self.travel_speed if self.phase == "travel" else self.dig_speed
        v, w = ctx.pp.command(ctx.pose, lookahead, v_max=v_max)
        ctx.cmd(v, w)
        if self.stuck.check(ctx, True):
            self.finish(ctx)
            self.lane.failures += 1
            return self.fail("stuck")
        return RUNNING

    def describe(self):
        return f"dig(strip {self.lane.index}, pass {self.lane.passes + 1}, {self.phase})"


class UnloadAtHopper(Action):
    """Empties one drum over the hopper bin. Reports how much of the load actually fell in the bin."""
    name = "unload"

    def __init__(self, drum: str, empty_kg: float = 0.1, timeout: float = 30.0) -> None:
        super().__init__()
        self.drum: str = drum              # front or rear
        self.empty: float = empty_kg       # how much kg in drums should be considered as empty
        self.timeout: float = timeout      # s
        self.haul = True

    def current_load(self, st):
        return st.get(f"{self.drum}_load_kg", 0.0)

    def start(self, ctx):
        super().start(ctx)
        ctx.set_implements(None, None, -1.0 if self.drum == "front" else 0.0, -1.0 if self.drum == "rear" else 0.0)

    def update(self, ctx):
        ctx.stop_base()
        st = ctx.state
        current_load = self.current_load(st)
        if (current_load <= self.empty and self.elapsed(ctx) > 1.0) or self.elapsed(ctx) > self.timeout:
            ctx.set_implements(None, None, 0.0, 0.0)
            return DONE
        return RUNNING

    def describe(self):
        return f"unload({self.drum} drum)"


class Charge(Action):
    """Waits at the antenna until the battery reaches target_soc."""
    name = "charge"

    def __init__(self, target_soc: float, timeout: float = 7200.0) -> None:
        super().__init__()
        self.target: float = target_soc      # 0-1
        self.timeout: float = timeout
        self.t_off_antenna: float | None = None

    def update(self, ctx):
        ctx.stop_base()
        st = ctx.state
        if st.get("soc", 0.0) >= self.target:
            return DONE
        
        if not st.get("docked", False):
            if self.t_off_antenna is None:
                self.t_off_antenna = ctx.now()
            elif ctx.now() - self.t_off_antenna > 6.0:
                return self.fail("not at the antenna")
        else:
            self.t_off_antenna = None

        if self.elapsed(ctx) > self.timeout:
            return self.fail("charge timeout")
        return RUNNING
