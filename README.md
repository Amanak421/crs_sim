# crs_sim

Standalone simulator of the **CRS97** robotic arm. It has the same interface as
the real robot's driver [`ctu_crs`](https://pypi.org/project/ctu-crs/), so code
written for the hardware runs on the simulator by changing one import:

```python
from ctu_crs import CRS97   # real robot
from crs_sim import CRS97   # simulator
```

The robot is rendered from its URDF + STL meshes in a [MeshCat](https://github.com/meshcat-dev/meshcat-python)
browser window.

## Installation

```bash
git clone <this repo> crs_sim && cd crs_sim
./setup_venv.sh            # creates .venv, installs crs_sim (editable) + pytest + STEP support
source .venv/bin/activate
```

`PYTHON=python3.11 ./setup_venv.sh` picks a different interpreter and
`EXTRAS=dev ./setup_venv.sh` skips the CAD/STEP support (~100 MB, OpenCASCADE). To use the
package from another environment (e.g. a conda env), run `pip install -e /path/to/crs_sim`.

Quick start:

```bash
./start.sh                  # interactive console with `robot` already created
./start.sh examples/demo.py # run a script inside the venv
./start.sh -m pytest        # tests (headless)
```

`start.sh` and `setup_venv.sh` unset `PYTHONPATH`, so a sourced ROS workspace
cannot leak its packages (and pytest plugins) into the venv. If you activate the
venv manually with ROS sourced, run `unset PYTHONPATH` first.

Dependencies: `numpy`, `pin` (pinocchio) and `meshcat`; the optional `step` extra
(`trimesh`, `cascadio`) loads CAD files. The real-robot packages (`ctu_crs`,
`pypylon`, serial port drivers) are **not** needed.

## Usage

```python
import numpy as np
from crs_sim import CRS97

robot = CRS97()                  # opens the viewer in the browser
robot.initialize()               # hard home + soft home, like the real robot

q = robot.q_home + [0.3, 0.2, 0.0, 0.0, 0.0, 0.5]
robot.move_to_q(q)               # non-blocking, exactly like the real robot
robot.wait_for_motion_stop()
print(robot.get_q())

pose = robot.fk(q)                                   # 4x4 flange pose
sols = [s for s in robot.ik(pose) if robot.in_limits(s)]
```

## Examples

Run them with `./start.sh examples/<name>.py`, all take `--help`.

| Example | What it shows |
| :--- | :--- |
| `demo.py` | Reach a target pose with IK, draw frames |
| `puzzle_a.py` | Puzzle A in the workspace, the hoop moved down the wire and back through fixed joint configurations |
| `show_cad.py [files...]` | Show any STEP/STL/OBJ file in the workspace; without arguments all five puzzles |
| `draw_circle.py` | Cartesian path following: trace a circle with IK, draw the trail |
| `joint_sweep.py` | Joint limits table and each joint swept towards its limits |
| `real_or_sim.py [--real]` | The same control code on the simulator or the real robot |

## Puzzles and CAD files

`crs_sim.puzzles` has the hoop offset `HOOP_IN_FLANGE` (hoop center pose =
`robot.fk(q) @ HOOP_IN_FLANGE`), `show_puzzle()` to put any of the puzzles A - E
into the scene, and the wire center line of puzzle A as an example
(`maze_path("A")`, hoop poses in the maze frame).

The CAD models come from [pilcsimo/ROB1-2025-hw](https://github.com/pilcsimo/ROB1-2025-hw)
(GPL-3.0). They are **not** included here; `download_puzzles()` fetches them to
`~/.cache/crs_sim/` on first use.

```python
from crs_sim import CRS97
from crs_sim.puzzles import show_puzzle
from crs_sim.transforms import pose

robot = CRS97()
show_puzzle(robot.viz, "D", pose([0.45, 0.0, 0.0], yaw=0.5))   # base plate + wire
robot.viz.show_mesh("part", "my_part.step", pose([0.3, 0.2, 0.0]))
robot.viz.show_mesh("bracket", "bracket.stl", scale=0.001)       # STL in mm
```

`crs_sim.transforms` has the small 4x4 helpers used above (`rx`, `ry`, `rz`,
`translation`, `pose`, `interpolate`, `densify`).

## Fidelity to the real robot

| | Real `ctu_crs.CRS97` | `crs_sim.CRS97` |
| :--- | :--- | :--- |
| `fk`, `ik`, `fk_flange_pos`, DH params | analytic DH model | identical code, identical results (IK solutions are shuffled too) |
| `q_min`, `q_max`, `q_home` | from IRC limits | identical |
| `move_to_q` | non-blocking, asserts `initialize()` was called and joint limits | same; the motion takes real time |
| `get_q`, `in_motion`, `wait_for_motion_stop` | read from the controller | follow the simulated motion, `q` quantized to encoder (IRC) resolution |
| `soft_home`, `hard_home`, `initialize(home=...)` | | same behavior, hard home moves the joints in the real `B → AC → DEF` order |
| `set_speed(_relative)` | | changes the simulated joint speeds |
| `set_acceleration(_relative)` | | stored only (a smooth quintic profile is used) |
| `gripper.*` | | tracks the commanded position, not rendered |
| `grab_image` | Basler camera | returns `robot.camera_image` (set it yourself), raises otherwise |
| `release`, `reset_motors`, `close` | | no-ops |

The URDF agrees with the DH model to < 1 mm. The rendered `adapter` link equals
the DH flange frame rotated by π about its y-axis.

## Simulator-only options

All of these are keyword arguments of `CRS97(...)`:

| Option | Default | Meaning |
| :--- | :--- | :--- |
| `visualize` | `True` | `False` runs headless (kinematics + timing only, no pinocchio/meshcat needed) |
| `open_browser` | `True` | open the MeshCat page automatically |
| `speedup` | `1.0` | time scale of motions; `np.inf` makes them instantaneous |
| `q0` | `q_home` | initial joint configuration |
| `render_fps` | `30` | render rate during motions |

## Drawing into the scene

`robot.viz` (a `crs_sim.Visualizer`) draws helper objects:

```python
robot.viz.show_points("path", points_nx3, color=0xffa500, radius=0.005)
robot.viz.show_frame("target", pose_4x4, scale=0.1)
robot.viz.show_box("table", [1.8, 0.8, 0.02], pose_4x4, opacity=0.3)
robot.viz.show_mesh("part", "part.step", pose_4x4)   # any CAD file, needs the step extra
robot.viz.set_camera([1.2, -1.4, 1.0])
robot.viz.delete("path")      # or robot.viz.delete() for all helper objects
robot.viz.viewer              # raw meshcat.Visualizer
```

## License

The kinematics and robot parameters are taken from `ctu_crs` (BSD 3-Clause,
Copyright (c) 2023, CTURobotics). See `LICENSE-ctu_crs`.
