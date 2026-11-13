"""
Contains recharging strategies and CycleEstimator for cycle duration
"""
from .geometry import Point, dist
from .site import SiteModel, Lane, antenna_stage

STRATEGIES = ("threshold", "predictive")


class CycleEstimator:
    """Approximate time estimates for one dig-transport-unload cycle."""

    def __init__(self, parameters: dict, site: SiteModel) -> None:
        self.parameters: dict = parameters
        self.site: SiteModel = site
        self.yield_kg_m: float = parameters["dig_rate_kg_s"] / parameters["dig_speed"]

    def lane_start(self, lane:Lane)->Point:
        """Returns the lane starting point."""
        return lane.point(lane.next_s - self.parameters["drum_offset"] - 0.2)

    def expected_advance(self, lane:Lane):
        """Metres to dig to fill both drums."""
        cap = 2.0 * self.parameters["drum_capacity_kg"]
        need = cap / self.yield_kg_m
        return min(need, max(lane.length - lane.next_s, 0.0))

    def lane_end(self, lane:Lane):
        """Returns the lane end point where the drums will be filled."""
        return lane.point(lane.next_s + self.expected_advance(lane) + self.parameters["drum_offset"])

    def hopper_stage(self):
        """Returns the hopper stage point."""
        return self.site.hopper.stage(self.parameters["hopper_stage_dist"])

    def travel_time(self, a:Point, b:Point):
        """Returns the estimated time required to go from a to b at v_max."""
        return dist(a, b) / (0.6 * self.parameters["v_max"]) + 10.0     # + turning or heading adjustment

    def dig_time(self, lane:Lane):
        """Returns the estimated time required to dig a lane ad dig_speed."""
        parameters = self.parameters
        time_to_dig = self.expected_advance(lane) / parameters["dig_speed"]
        time_to_advance_offsets = 2.0 * parameters["drum_offset"] / (2.0 * parameters["dig_speed"]) 
        time_for_arms = 10.0
        return time_to_dig + time_to_advance_offsets + time_for_arms

    def unload_time(self):
        """Returns the estimated time required to approach the hopper and unload both drums."""
        parameters = self.parameters
        time_to_reach_hopper = parameters["hopper_stage_dist"] / parameters["approach_speed"]
        time_to_turn_around = 2.0 * parameters["hopper_turn_dist"] / parameters["approach_speed"]
        time_to_leave_hopper = parameters["hopper_stage_dist"] / 0.12
        time_to_unload_drums = 2.0 * parameters["drum_capacity_kg"] / parameters["dump_rate_kg_s"]
        time_to_move_arms = 20
        return time_to_reach_hopper + time_to_turn_around + time_to_leave_hopper + time_to_unload_drums + time_to_move_arms

    def cycle_time(self, pose_xy, lane:Lane):
        """Returns the estimated time required to execute a whole cycle covering lane passed in parameters."""
        time_pose_lane_start = self.travel_time(pose_xy, self.lane_start(lane))
        time_digging = self.dig_time(lane)
        time_lane_end_hopper = self.travel_time(self.lane_end(lane), self.hopper_stage())
        time_unload = self.unload_time()
        return time_pose_lane_start + time_digging + time_lane_end_hopper + time_unload

    def antenna_detour_time(self):
        """Returns the estimated time required to go from the hopper to the charger and back."""
        parameters = self.parameters
        time_to_charger = self.travel_time(self.hopper_stage(), antenna_stage(parameters))
        time_to_antenna = parameters["antenna_stage_dist"] / parameters["approach_speed"]
        return 2.0 * time_to_charger + 2.0 * time_to_antenna


def charge_decision(strategy: str, st: dict, pose_xy, lane:Lane|None, est:CycleEstimator, params:dict, avg_power_w:float, time_left_s:float):
    """Decides whether or not recharge before the next cycle and if so how much. Returns None if no recharge is required, target SoC otherwise."""
    parameters = params
    soc = st.get("soc", 1.0)
    capacity_wh = max(st.get("capacity_wh", 1.0), 1e-6)

    if strategy == "threshold":
        return parameters["soc_full"] if soc < parameters["soc_return"] else None

    # predictive
    energy_wh = st.get("energy_wh", soc * capacity_wh)
    reserve = parameters["soc_reserve"] * capacity_wh
    t_next = est.cycle_time(pose_xy, lane) if lane is not None else 0.0
    t_back = est.antenna_detour_time() / 2.0 + 30.0
    energy_needed = avg_power_w * (t_next + t_back) / 3600.0 * parameters["energy_margin"]
    if energy_wh - energy_needed >= reserve:
        return None

    charge_power_w = parameters["charge_power_w"]

    # energy the rover would use working for all the time left
    energy_for_rest_wh = avg_power_w * time_left_s / 3600.0 * parameters["energy_margin"]
    energy_missing_wh = max(0.0, energy_for_rest_wh + reserve - energy_wh)

    # charging E Wh takes E / charge_power_w hours and in that time the rover does not work so it will not need the energy of that time either
    # E = energy_missing_wh - avg_power_w * E / charge_power_w
    energy_to_charge_wh = energy_missing_wh / (1.0 + avg_power_w / charge_power_w)
    target_soc = (energy_wh + energy_to_charge_wh) / capacity_wh

    # avoid charging just enough to go back to cycle starting point and realize not enough SoC
    min_soc = (energy_needed + reserve) / capacity_wh + 0.05
    return min(parameters["soc_full"], max(min_soc, target_soc))
