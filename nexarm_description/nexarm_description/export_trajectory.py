"""Export the current inverse-kinematics trajectory without starting ROS."""
import argparse
import csv
import math
from pathlib import Path

from .draw_s import chain_from_xml, plan


def main():
    """Save only time and joint angles in degrees as CSV."""
    from ament_index_python.packages import get_package_share_directory
    import xacro

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='trajectories/ee528.csv')
    parser.add_argument('--height', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=50.0)
    parser.add_argument('--lift', type=float, default=0.015)
    args = parser.parse_args()
    model = Path(get_package_share_directory('nexarm_description')) / 'urdf/nexarm.urdf.xacro'
    xml = xacro.process_file(str(model)).toxml()
    positions, _, _ = plan(chain_from_xml(xml), args.height, args.duration, args.lift)
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s'] + [f'joint{i}_deg' for i in range(1, 6)])
        for index, q in enumerate(positions):
            angles = [*map(float, q), 0.0]
            writer.writerow([f'{index / 30:.6f}']
                            + [f'{math.degrees(a):.6f}' for a in angles])
    print(f'{output}: {len(positions)} samples, {(len(positions)-1)/30:.2f} seconds, 30 Hz')
    print('Angles use URDF joint coordinates, not calibrated servo encoder values.')


if __name__ == '__main__':
    main()
