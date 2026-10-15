"""
Standalone simulator of the CRS97 robotic arm with the interface of ``ctu_crs``.

    from crs_sim import CRS97      # instead of: from ctu_crs import CRS97
"""
from .crs97 import CRS97, CRS97_PARAMS, URDF_PATH
from .crs_robot import CRSRobot
from .visualizer import Visualizer, load_mesh

__all__ = ["CRS97", "CRS97_PARAMS", "CRSRobot", "URDF_PATH", "Visualizer", "load_mesh"]
