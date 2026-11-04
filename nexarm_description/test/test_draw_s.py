"""Check the complete drawing against model limits and the horizontal plane."""
from pathlib import Path

import numpy as np
import pytest

from nexarm_description.draw_s import chain_from_xml, forward, plan


def test_horizontal_s():
    model = Path(__file__).parents[1] / 'urdf' / 'nexarm.urdf.xacro'
    chain = chain_from_xml(model.read_text())
    q, tips = plan(chain)
    assert np.max(np.abs(tips[:, 2])) < 0.0005
    assert np.min(tips[:, 0]) == pytest.approx(0.235, abs=0.0005)
    assert np.ptp(tips[:, 0]) == pytest.approx(0.07, abs=0.0005)
    assert np.ptp(tips[:, 1]) == pytest.approx(0.14, abs=0.0005)
    middle = len(tips) // 2
    assert np.allclose(tips[middle], [0.27, 0., 0.], atol=0.0005)
    # Each lobe follows a circle, rather than a shallow sine wave.
    for lobe, center in [(tips[:middle + 1], [0.27, 0.035]),
                         (tips[middle:], [0.27, -0.035])]:
        radii = np.linalg.norm(lobe[:, :2] - center, axis=1)
        assert np.allclose(radii, 0.035, atol=0.0005)
    incoming = tips[middle] - tips[middle - 1]
    outgoing = tips[middle + 1] - tips[middle]
    assert np.dot(incoming, outgoing) / (np.linalg.norm(incoming)
                                       * np.linalg.norm(outgoing)) > 0.99
    assert np.max(np.abs(np.diff(q, axis=0))) * 30 < 3.14
    for angles, tip in zip(q, tips):
        pose = forward(chain, angles)
        assert np.allclose(pose[:3, 2], [0, 0, 1], atol=0.005)
        assert np.allclose(tip, pose[:3, 3] - 0.10 * pose[:3, 2])
    for i, joint in enumerate(chain[:4]):
        assert np.all(q[:, i] >= float(joint[3].get('lower')))
        assert np.all(q[:, i] <= float(joint[3].get('upper')))
    with pytest.raises(ValueError):
        plan(chain, height=2.0)
    with pytest.raises(ValueError):
        plan(chain, duration=0.0)
