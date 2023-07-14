import os
from setuptools import setup, find_packages
from glob import glob

PACKAGE_NAME = 'flex_nav_flexbe_states'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(),
    data_files=[
     (os.path.join('share', PACKAGE_NAME), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + "/tests", glob('tests/*.test')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='david.conner@cnu.edu',
    description='flex_nav_flexbe_states provides a collection of FlexBE states used during the SSP Research',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
