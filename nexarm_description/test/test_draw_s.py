"""Verify writing geometry, pen lifts, and actual joint constraints."""
from pathlib import Path

import numpy as np
import pytest

from nexarm_description.draw_s import (
    chain_from_xml, forward, plan, text_strokes, writing_targets,
)


def test_writing_and_pen_lifts():
    model = Path(__file__).parents[1] / 'urdf' / 'nexarm.urdf.xacro'
    chain = chain_from_xml(model.read_text())
    e = text_strokes()[0]
    # The tail reaches the outer right edge while leaving the e open.
    assert abs(e[-1, 1] - np.min(e[:, 1])) < 0.001
    assert e[-1, 0] < e[0, 0] - 0.005
    eight = text_strokes()[-1]
    assert np.allclose(eight[0], eight[-1])
    assert np.allclose(eight[0], eight[len(eight) // 2])
    assert np.ptp(eight[:, 0]) > 0.04
    targets, expected_ink = writing_targets()
    q, tips, ink = plan(chain)
    assert len(text_strokes()) == 5  # Each character is a single continuous stroke.
    assert np.array_equal(ink, expected_ink)
    assert np.allclose(tips, targets, atol=0.0005)
    assert tips[-1, 2] == pytest.approx(0.015, abs=0.0005)
    assert np.max(np.abs(np.diff(q, axis=0))) * 30 < 3.14
    for angles, tip in zip(q, tips):
        pose = forward(chain, angles)
        assert np.allclose(pose[:3, 2], [0, 0, 1], atol=0.005)
        assert np.allclose(tip, pose[:3, 3] - 0.10 * pose[:3, 2])
    for i, joint in enumerate(chain[:4]):
        assert np.all(q[:, i] >= float(joint[3].get('lower')))
        assert np.all(q[:, i] <= float(joint[3].get('upper')))
    # Rendered segments touch the paper at both ends; no travel gets ink.
    edges = np.flatnonzero(ink)
    assert np.allclose(tips[edges, 2], 0, atol=0.0005)
    assert np.allclose(tips[edges - 1, 2], 0, atol=0.0005)
    starts = np.flatnonzero(ink & ~np.r_[False, ink[:-1]])
    ends = np.flatnonzero(ink & ~np.r_[ink[1:], False])
    assert len(starts) == len(ends) == 5
    for end, start in zip(ends[:-1], starts[1:]):
        travel = targets[end:start]
        assert np.max(travel[:, 2]) == pytest.approx(0.015)
        # Horizontal movement between strokes only occurs at lifted height.
        moving = np.linalg.norm(np.diff(travel[:, :2], axis=0), axis=1) > 1e-9
        assert np.allclose(travel[:-1][moving, 2], 0.015)
    with pytest.raises(ValueError):
        plan(chain, height=2.0)
    with pytest.raises(ValueError):
        plan(chain, duration=0.0)
    with pytest.raises(ValueError):
        writing_targets(lift=0)
