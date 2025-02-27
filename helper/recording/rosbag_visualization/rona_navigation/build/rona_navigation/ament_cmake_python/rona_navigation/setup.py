from setuptools import find_packages
from setuptools import setup

setup(
    name='rona_navigation',
    version='0.0.0',
    packages=find_packages(
        include=('rona_navigation', 'rona_navigation.*')),
)
