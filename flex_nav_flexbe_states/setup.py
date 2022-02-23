import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'flex_nav_flexbe_states'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(),
    data_files=[
     (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
