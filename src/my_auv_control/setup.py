from setuptools import setup
import os
from glob import glob

package_name = 'my_auv_control'

setup(
    name=package_name,
    version='0.2.0',
    packages=['my_auv_control', 'auv_nav'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='golfe',
    maintainer_email='your_email@example.com',
    description='AUV Control, OOP Autopilot & Utilities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopilot         = auv_nav.autopilot_node:main',
            'autopilot49       = my_auv_control.auv_pid_nav:main',
            'fake_barometer    = my_auv_control.fake_barometer:main',
            'mixer             = my_auv_control.auv_control_mixer:main',
            'mixer_nb          = my_auv_control.mixer_nb:main',
            'alt_ballast       = my_auv_control.alt_ballast_control:main',
            'mass_calibration  = my_auv_control.mass_calibration:main',
            'ballast_neutral   = my_auv_control.ballast_neutral_finder:main',
            'test_straight     = my_auv_control.auv_test_straight:main',
            'test_telemetry    = my_auv_control.test_straight:main',
        ],
    },
)
