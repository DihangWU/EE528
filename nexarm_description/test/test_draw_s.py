"""Verify writing geometry, pen lifts, and actual joint constraints."""
from pathlib import Path

import numpy as np
import pytest

from nexarm_description.draw_s import (
    GRIPPER_HOLD, PEN_FORWARD, PEN_LENGTH, chain_from_xml, forward, plan, text_strokes, writing_targets,
)


def test_writing_and_pen_lifts():
    model = Path(__file__).parents[1] / 'urdf' / 'nexarm.urdf.xacro'
    chain = chain_from_xml(model.read_text())
    e = text_strokes()[0]
    # The tail reaches the outer right edge while leaving the e open.
    assert abs(e[-1, 1] - np.min(e[:, 1])) < 0.001
    assert e[-1, 0] < e[0, 0] - 0.005
    eight = text_strokes()[4]
    assert np.allclose(eight[0], eight[-1])
    assert np.allclose(eight[0], eight[len(eight) // 2])
    assert np.ptp(eight[:, 0]) > 0.04
    targets, expected_ink = writing_targets()
    q, tips, ink = plan(chain)
    strokes = text_strokes()
    assert len(strokes) == 18  # Five first-line strokes and thirteen name strokes.
    top, bottom = np.vstack(strokes[:5]), np.vstack(strokes[5:])
    # Both lines share exactly the same left/right ink bounds.
    assert np.allclose([top[:, 1].min(), top[:, 1].max()],
                       [bottom[:, 1].min(), bottom[:, 1].max()])
    assert bottom[:, 0].max() < top[:, 0].min() - .01
    assert np.array_equal(ink, expected_ink)
    # Robot mount is 10 cm above world ground; ink lies on world z=0.
    import xml.etree.ElementTree as ET
    root = ET.parse(model).getroot()
    mount = root.find("joint[@name='world_to_base_joint']/origin")
    mount_z = float(mount.get('xyz').split()[2])
    assert mount_z == pytest.approx(0.10)
    assert np.allclose(tips[ink, 2] + mount_z, 0., atol=0.0005)
    # Original front writing, with no 90-degree offset at J1.
    assert np.allclose(targets[0, :2], [.3775, .085])
    assert np.all(tips[:, 0] >= .290)
    assert np.all(np.abs(q[:, 0]) < .4)
    assert np.allclose(tips, targets, atol=0.0005)
    assert tips[-1, 2] == pytest.approx(-0.085, abs=0.0005)
    assert np.max(np.abs(np.diff(q, axis=0))) * 30 < 3.14
    # Longer pen exactly compensates the raised mount at the world ground.
    assert PEN_LENGTH == pytest.approx(0.125)
    gripper_heights = np.array([forward(chain, angles)[2, 3] for angles in q])
    assert np.allclose(gripper_heights[ink], 0.025, atol=0.0005)
    # The RViz pen is an unconditional rigid robot link, not a delayed Marker.
    pen_link = root.find("link[@name='pen_link']")
    assert pen_link is not None
    pen_joint = root.find("joint[@name='pen_joint']")
    assert pen_joint.get('type') == 'fixed'
    assert pen_joint.find('parent').get('link') == 'gripper_link'
    assert np.allclose(np.fromstring(pen_joint.find('origin').get('xyz'), sep=' '),
                       [PEN_FORWARD, 0, 0])
    pen_visual = pen_link.find('visual')
    assert float(pen_visual.find('geometry/cylinder').get('length')) == PEN_LENGTH
    # Opposing inner fingertip surfaces meet the 6 mm pen, not an open grip.
    left_inner = 0.038561 - 0.0131701054 - 0.05 * GRIPPER_HOLD
    assert left_inner == pytest.approx(0.003)
    assert 0 < GRIPPER_HOLD <= 0.5
    import yaml
    config = yaml.safe_load((model.parents[1] / 'rviz/draw_s.rviz').read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'world'
    grid = next(d for d in manager['Displays'] if d['Class'].endswith('/Grid'))
    assert grid['Offset']['Z'] == 0
    for angles, tip in zip(q, tips):
        pose = forward(chain, angles)
        assert np.allclose(pose[:3, 2], [0, 0, 1], atol=0.005)
        assert np.allclose(tip, pose[:3, 3] + PEN_FORWARD * pose[:3, 0] - PEN_LENGTH * pose[:3, 2])
    for i, joint in enumerate(chain[:4]):
        assert np.all(q[:, i] >= float(joint[3].get('lower')))
        assert np.all(q[:, i] <= float(joint[3].get('upper')))
    # Rendered segments touch the paper at both ends; no travel gets ink.
    edges = np.flatnonzero(ink)
    assert np.allclose(tips[edges, 2], -0.10, atol=0.0005)
    assert np.allclose(tips[edges - 1, 2], -0.10, atol=0.0005)
    starts = np.flatnonzero(ink & ~np.r_[False, ink[:-1]])
    ends = np.flatnonzero(ink & ~np.r_[ink[1:], False])
    assert len(starts) == len(ends) == len(strokes)
    for end, start in zip(ends[:-1], starts[1:]):
        travel = targets[end:start]
        assert np.max(travel[:, 2]) == pytest.approx(-0.085)
        # Horizontal movement between strokes only occurs at lifted height.
        moving = np.linalg.norm(np.diff(travel[:, :2], axis=0), axis=1) > 1e-9
        assert np.allclose(travel[:-1][moving, 2], -0.085)
    with pytest.raises(ValueError):
        plan(chain, height=2.0)
    with pytest.raises(ValueError):
        plan(chain, duration=0.0)
    with pytest.raises(ValueError):
        writing_targets(lift=0)
