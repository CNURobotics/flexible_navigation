"""Setup for pure pursuit."""
from setuptools import setup

PACKAGE_NAME = 'flex_nav_pure_pursuit'

setup(
    name=PACKAGE_NAME,
    version='1.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='david.conner@cnu.edu',
    description='Implements Pure Pursuit algorithm to be used for following a path',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_path = flex_nav_pure_pursuit.pure_pursuit_path:main',
            'pure_pursuit_topic = flex_nav_pure_pursuit.pure_pursuit_topic:main'
        ],
    },
)
