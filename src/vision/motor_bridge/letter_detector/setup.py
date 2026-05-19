from setuptools import setup
from glob import glob
import os

package_name = 'letter_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'templates'),
            glob('letter_detector/templates/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Letter detector package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = letter_detector.detector_node:main',
            'camera_subscriber = letter_detector.camera_subscriber:main',
            'alphabet_detector = letter_detector.v2:main',
        ],
    },
)
