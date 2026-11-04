from setuptools import find_packages, setup

setup(
    name='nexarm_mpc', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/nexarm_mpc']),
                ('share/nexarm_mpc', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='maple', maintainer_email='liangfuyuan@example.com',
    description='Linear Constrained Incremental MPC for joint tracking', license='Apache-2.0',
)
