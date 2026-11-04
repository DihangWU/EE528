from setuptools import find_packages, setup

setup(name='nexarm_lqr', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/nexarm_lqr']),
                  ('share/nexarm_lqr', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='maple', maintainer_email='liangfuyuan@example.com',
      description='LQR joint tracking simulation controller', license='Apache-2.0')
