from setuptools import find_packages
from setuptools import setup

setup(
    name='pm_skills_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('pm_skills_interfaces', 'pm_skills_interfaces.*')),
)
