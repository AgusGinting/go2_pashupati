from __future__ import annotations

from glob import glob

from setuptools import find_packages, setup

package_name = 'go2_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml') + glob('config/*.json')),
        (f'share/{package_name}/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Pashupati robot package for the Unitree Go2 EDU.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go2_driver_node = go2_bringup.go2_driver_node:main',
        ],
    },
)
