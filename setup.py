import os
from glob import glob

from setuptools import setup

package_name = 'ward_patrol'


def tree(src):
    """ディレクトリ構造を保ったまま share/ にインストールするための data_files."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/weights', glob('weights/*.pt')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ytkw378',
    maintainer_email='ytkw378@users.noreply.github.com',
    description='AI night patrol robot that finds fallen patients in hospital rooms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fall_detector = ward_patrol.fall_detector:main',
            'patrol_manager = ward_patrol.patrol_manager:main',
            'ward_monitor = ward_patrol.ward_monitor:main',
        ],
    },
)
