"""Frob City core navigation package.

Pure domain modules such as :mod:`models` and :mod:`config_loader` deliberately do
not import ROS. ROS wrappers are added by later work packages.
"""

from .models import CONTRACT_VERSION

__all__ = ["CONTRACT_VERSION"]
