# IPEx Lunar Sim
![Unity](https://img.shields.io/badge/Unity-000000?style=flat&logo=unity&logoColor=white)
![C#](https://img.shields.io/badge/C%23-512BD4?style=flat&logo=dotnet&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS_2-22314E?style=flat&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat&logo=opencv&logoColor=white)

![Scene overview](.github/readme_assets/scene_overview.png)

This project simulates the autonomous operation of the IPEx rover in Unity with ROS2. The rover digs regolith from the surface, hauls it to a hopper and manages its state of charge.
<center>
    <video width="1920" height="720" style controls>
      <source src=".github/readme_assets/demo_video_compressed.mp4" type="video/mp4">
    </video>
</center>

The repository contains:
- `ipex_lunar_sim`: the Unity project containing the virtual scene.
- `ros2_workspace`: the ROS2 workspace with the mission logic.

## Unity Setup

The project requires Unity 6000.3.8f1 or later. The ROS2 integration has so far been tested only on Windows, although it can also be set up to work on Linux (see [ROS2-for-Unity](https://github.com/RobotecAI/ros2-for-unity/blob/feature/jazzy-support/README-UBUNTU.md)).

ros2-for-unity, built in *standalone* mode for Windows, is already included in the repository.
To build it on Windows, assuming ROS2 has been installed in `C:\pixi_ws\ros2-windows` following the official ROS2 guide:
```ps
cd C:\pixi_ws
pixi shell # Enable Pixi Shell
ros2-windows\local_setup.ps1 # Source ROS2
```
Then activate the x64 Native Tools Command Prompt environment (requires Visual Studio 2022 or later):
```ps
Import-Module "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"

Enter-VsDevShell -VsInstallPath "C:\Program Files\Microsoft Visual Studio\2022\Community" -SkipAutomaticLocation -DevCmdArguments "-arch=x64 -host_arch=x64"
```
Then, in a folder preferably located at the root of a drive (e.g. `C:\` or `D:\`), run:
```ps
git clone -b feature/jazzy-support https://github.com/RobotecAI/ros2-for-unity
cd ros2-for-unity
.\pull_repositories.ps1

cp -Recurse ros2_workspace\src\ipex_interfaces src\ros2cs\src
.\build.ps1 -standalone
```
The first build takes about 30 minutes. When it finishes, copy `ros2-for-unity\install\asset\Ros2Unity` into the `Assets` folder of the Unity project.

Finally, open the `ipex_lunar_sim` folder with Unity Hub, then open the scene `Assets/Scenes/MainScene.unity`.

In Play mode, the rover can be controlled manually:

| Key | Function |
|-------|----------|
| W / S | Forward / backward |
| A / D | Steering |
| Space / X | Dig / Unload |
| I / K | Raise / lower arms |

> If you move anything in the scene other than the rover, run **IPEx Mission > Export Survey**. This generates a JSON file in the project folder: copy it into the `ros2_workspace/src/ipex_mission/config` directory, overwriting the existing one and then rebuild.

> If built from "Build and run" or from "Build profiles" and you notice heavy frame rate drops or glitching textures after a while, that means there are system DLLs or libs that are overriding the game ones. On Windows this can be solved by running the game with empty PATH environment varialbe via `$env:PATH = "$env:SystemRoot\System32;$env:SystemRoot"; & 'C:\path\to\ipex_lunar_sim.exe'`.

## ROS2 Packages



The `ipex_mission` package requires ROS2 (Jazzy distribution). Its dependencies are listed in `package.xml`. Running it on Linux is recommended, as installing the dependencies is easier there.

Build both `ipex_interfaces` and `ipex_mission` (assuming Linux environment):

```bash
cd ros2_workspace
colcon build --merge-install
source install/setup.bash
```

To start the mission, with Unity scene in Play mode:

```bash
ros2 launch ipex_mission mission_launch.py strategy:=predictive rviz:=true
```

Main arguments: `strategy` (`threshold` or `predictive`), `max_mission_time`, `localization` (`tags` or `ground_truth`), `rviz`.

Many mission parameters are defined in `ros2_workspace/src/ipex_mission/config/mission.yaml`. Their meaning is reported here:

### Common (`/**`)

| Parameter | Default | Meaning |
|---|---|---|
| `use_sim_time` | `true` | Use the `/clock` published by Unity instead of the wall clock |
| `cameras` | 6 cameras | Cameras used for tag detection (`front/left`, `front/right`, `rear/left`, `rear/right`, `side/left`, `side/right`) |

### Mission (`mission`)

| Parameter | Default | Meaning |
|---|---|---|
| `strategy` | `predictive` | Recharging strategy: `threshold` or `predictive` |
| `max_mission_time` | `7200.0` s | Mission length in simulated seconds |
| `target_delivered_kg` | `0` | Stop after delivering this much regolith (`0` = run until the time limit) |
| `pose_ready_std_m` | `0.5` m | The EKF pose is used only once its standard deviation is below this |
| `area_length` | `6.0` m | Area length along the digging direction |
| `area_width` | `3.0` m | Area width across the digging direction |
| `lane_spacing` | `0.5` m | Distance between digging lanes |
| `max_passes` | `2` | How many times the same lane is dug |
| `area_r_max` | `13.0` m | Maximum distance of the area from the lander |
| `area_grid` | `1.0` m | Spacing of the grid for area evaluation |
| `area_clearance` | `1.2` m | Minimum distance of the area from lander, hopper and antenna approaches |
| `max_drive_slope_deg` | `12.0` ° | Terrain steeper than this is not driven on (cost map) |
| `hopper_stage_dist` | `3.0` m | Length of the straight final approach to the hopper |
| `hopper_standoff` | `0.5` m | Distance between rover center and hopper wall when unloading |
| `hopper_turn_dist` | `1.0` m | How far the rover backs out to turn around and unload the rear drum |
| `approach_speed` | `0.08` m/s | Speed on the final approaches to hopper and antenna |
| `min_haul_kg` | `1.0` kg | The rover does not drive to the hopper with less than this in the drums |
| `v_max` | `0.25` m/s | Maximum linear speed |
| `w_max` | `0.6` rad/s | Maximum angular speed |
| `lookahead` | `0.6` m | Pure pursuit lookahead distance |
| `inflation` | `0.55` m | Obstacles are grown by this in the cost map |
| `dig_speed` | `0.05` m/s | Forward speed while digging |
| `dig_rate_kg_s` | `1.5` kg/s | Maximum dig rate of one drum |
| `dig_arm_deg` | `-10.0` ° | Arm angle while digging (fully lowered) |
| `transport_arm_deg` | `55.0` ° | Arm angle while driving (fully raised) |
| `dump_arm_deg` | `45.0` ° | Arm angle while unloading into the hopper |
| `drum_offset` | `0.59` m | Distance of each drum center ahead / behind the rover center |
| `drum_capacity_kg` | `15.0` kg | Capacity of one drum |
| `dump_rate_kg_s` | `3.0` kg/s | Unloading rate of one drum |
| `antenna_side_gap` | `0.10` m | Gap between the rover side and the charging antenna when charging |
| `rover_half_width` | `0.267` m | Half of the rover width, wheels included |
| `antenna_charger_side` | `left` | Side of the rover that faces the antenna when charging (`left` or `right`) |
| `antenna_stage_dist` | `3.0` m | Length of the straight final approach to the antenna |
| `charge_power_w` | `1000.0` W | Antenna charging power |
| `soc_return` | `0.35` | `threshold`: go to recharge below this state of charge |
| `soc_full` | `0.95` | `threshold`: recharge up to this state of charge |
| `soc_reserve` | `0.10` | `predictive`: state of charge that must remain after the next cycle |
| `energy_margin` | `1.3` | `predictive`: safety factor for the next cycle |

### CLAHE (`clahe`)

| Parameter | Default | Meaning |
|---|---|---|
| `enabled` | `true` | If `false`, `apriltag_ros` reads `image_raw` directly |
| `clip` | `4.0` | CLAHE clip limit (see OpenCV `createCLAHE`) |
| `tiles` | `8` | CLAHE grid size (`tiles` × `tiles`) |

### Tag localizer (`tag_localizer`)

| Parameter | Default | Meaning |
|---|---|---|
| `base_frame` | `base_link` | Name of the base frame |
| `max_range_m` | `7.0` m | Reject pose if the nearest tag is further than this |
| `single_tag_max_range_m` | `3.0` m | Maximum distance at which a single tag is accepted |
| `max_rms_px` | `2.0` px | Maximum reprojection error of an accepted pose |
| `max_ambiguity` | `0.5` | Maximum ratio between the errors of the two possible planar poses (closer to 1 = more ambiguous) |
| `sigma0_m`, `sigma_k` | `0.005` m, `0.0035` | Position std dev sent to the EKF |
| `sigma_yaw0_deg`, `sigma_min_dist_m` | `0.2` °, `0.5` m | Heading std dev |

For `ekf` and `apriltag` sections see respectively: 
- [`robot_localization` documentation](https://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html)
- [`apriltag_ros` documentation](https://index.ros.org/p/apriltag_ros/#jazzy)