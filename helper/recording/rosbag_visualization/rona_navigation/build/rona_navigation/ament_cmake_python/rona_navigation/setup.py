from setuptools import find_packages
from setuptools import setup

setup(
    name='rona_navigation',
    version='0.0.0',
    packages=find_packages(
        include=('rona_navigation', 'rona_navigation.*')),
    entry_points={
        'console_scripts': [
            'timestamp_publisher = rona_navigation.timestamp_publisher:main',
        ],
    },
)
