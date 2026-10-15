"""
The ROB1 hoop-and-wire puzzles (A - E): CAD models and the wire center line
of puzzle A.

Each puzzle is a bent wire on a base plate. The task is to thread the hoop
mounted on the robot over the wire from its free end down to the base and back
without touching it. The CAD models (STEP) come from
https://github.com/pilcsimo/ROB1-2025-hw (GPL-3.0) and are downloaded on demand
by :func:`download_puzzles`, they are not part of this package.

Frames: a maze frame has its origin in the center of the base plate top, z up.
Paths are hoop-center poses whose z axis is tangent to the wire.
"""
from __future__ import annotations

import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np

from .transforms import densify, rx, rz, translation

#: Hoop center in the flange frame returned by ``CRS97.fk``. The wire passes
#: through the hoop along this frame's z axis.
HOOP_IN_FLANGE = translation([-0.135, 0.0, 0.012])

#: Base plate thickness, the puzzle stands on its top face [m]
BASE_PLATE_THICKNESS = 0.0056

PUZZLES_REPO_URL = "https://github.com/pilcsimo/ROB1-2025-hw"
_RAW_URL = "https://raw.githubusercontent.com/pilcsimo/ROB1-2025-hw/main/"
PUZZLE_FILES = {
    **{name: f"puzzles/{name}.step" for name in "ABCDE"},
    "base": "bases/base 01-02.step",
    "hoop": "robot_ee/Hoop.step",
}


def default_cache_dir() -> Path:
    """Directory the CAD files are downloaded to."""
    return Path.home() / ".cache" / "crs_sim" / "ROB1-2025-hw"


def download_puzzles(cache_dir: str | Path | None = None,
                     names: list[str] | None = None) -> dict[str, Path]:
    """
    Download the STEP models of the puzzles (skips files already present).
    :param cache_dir: target directory, :func:`default_cache_dir` if None
    :param names: subset of PUZZLE_FILES keys, all if None
    :return: name -> local path
    """
    cache_dir = Path(cache_dir) if cache_dir is not None else default_cache_dir()
    paths = {}
    for name in names or PUZZLE_FILES:
        rel = PUZZLE_FILES[name]
        path = cache_dir / rel
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            url = _RAW_URL + urllib.parse.quote(rel)
            print(f"Downloading {url}")
            tmp = path.with_suffix(".part")
            urllib.request.urlretrieve(url, tmp)
            tmp.rename(path)
        paths[name] = path
    return paths


# ----------------------------------------------------------------------
# Wire center lines
# ----------------------------------------------------------------------
# Segments, applied in order:
#   ("start", xyz, rotation)  first hoop pose (maze frame)
#   ("line", dxyz, n)         move by dxyz in the current hoop frame, n poses
# The current hoop frame has z pointing back along the wire, so "up the wire"
# is -z. Only puzzle A is provided, as an example of the format.
MAZES: dict[str, list[tuple]] = {
    "A": [
        ("start", [0, 0, 0.01], rz(np.pi) @ rx(np.pi)),
        ("line", [0, 0, -0.10], 2),
        ("line", [0, 0, -0.11], 2),
    ],
}


def maze_path(name: str) -> np.ndarray:
    """
    Hoop-center poses along the wire, from the base plate to past the free
    end, in the maze frame.
    :param name: key of MAZES ("A")
    :return: Nx4x4 poses
    """
    poses: list[np.ndarray] = []
    for seg in MAZES[name]:
        kind = seg[0]
        if kind == "start":
            start = translation(seg[1])
            start[:3, :3] = seg[2][:3, :3]
            poses.append(start)
        elif kind == "line":
            step = translation(np.asarray(seg[1], dtype=float) / seg[2])
            for _ in range(seg[2]):
                poses.append(poses[-1] @ step)
        else:
            raise ValueError(f"Unknown segment {kind}")
    # drop repeated poses
    keep = [0] + [i for i in range(1, len(poses))
                  if not np.allclose(poses[i], poses[i - 1])]
    return np.array(poses)[keep]


def show_puzzle(viz, name: str, maze_pose: np.ndarray, cad: bool = True,
                path: bool = True, color: int = 0xffa500) -> None:
    """
    Draw a puzzle into the scene: base plate and wire from the STEP models
    (downloaded on first use) and/or the wire center line.
    :param viz: crs_sim.Visualizer (``robot.viz``)
    :param name: "A" - "E"
    :param maze_pose: 4x4 pose of the maze frame in the robot base frame
    :param cad: draw the CAD models (needs the ``step`` extra)
    :param path: draw the wire center line as points (if it is in MAZES)
    :param color: color of the wire
    """
    if cad:
        files = download_puzzles(names=[name, "base"])
        viz.show_mesh(f"puzzle_{name}/wire", files[name], maze_pose, color=color)
        viz.show_mesh(f"puzzle_{name}/base", files["base"],
                      maze_pose @ translation([0, 0, -BASE_PLATE_THICKNESS]),
                      color=0xf0f0f0)
    if path and name in MAZES:
        points = (maze_pose @ densify(maze_path(name), 0.01))[:, :3, 3]
        viz.show_points(f"puzzle_{name}/path", points, color=0xff0000,
                        radius=0.003)
