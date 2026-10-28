"""Unit tests for the particle filter's coordinate helpers (no ROS graph needed)."""

import math

import numpy as np
import pytest
from nav_msgs.msg import MapMetaData

from particle_filter import utils


def make_map_info(resolution, x, y, yaw):
    info = MapMetaData()
    info.resolution = resolution
    info.origin.position.x = x
    info.origin.position.y = y
    info.origin.orientation = utils.angle_to_quaternion(yaw)
    return info


@pytest.mark.parametrize('angle', [0.0, 0.5, -1.2, math.pi - 0.01, -math.pi + 0.01])
def test_angle_quaternion_round_trip(angle):
    assert utils.quaternion_to_angle(utils.angle_to_quaternion(angle)) == pytest.approx(angle)


def test_map_to_world_scales_and_translates():
    info = make_map_info(resolution=0.05, x=-10.0, y=2.5, yaw=0.0)
    poses = np.array([[0.0, 0.0, 0.0], [100.0, 40.0, 0.3]])
    utils.map_to_world(poses, info)
    np.testing.assert_allclose(poses[0], [-10.0, 2.5, 0.0])
    np.testing.assert_allclose(poses[1], [-5.0, 4.5, 0.3])


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.0])
def test_world_to_map_inverts_map_to_world(yaw):
    info = make_map_info(resolution=0.05, x=-3.0, y=7.0, yaw=yaw)
    original = np.array([[12.0, 34.0, 0.1], [250.0, 5.0, -1.0], [0.0, 0.0, 2.0]])
    poses = original.copy()
    utils.map_to_world(poses, info)
    utils.world_to_map(poses, info)
    np.testing.assert_allclose(poses, original, atol=1e-9)
