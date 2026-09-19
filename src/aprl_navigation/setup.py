from glob import glob
from setuptools import setup

setup(
    name="aprl_navigation",
    version="0.1.0",
    packages=["aprl_navigation"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/aprl_navigation"]),
        ("share/aprl_navigation", ["package.xml"]),
        ("share/aprl_navigation/launch", glob("launch/*.launch.py")),
        ("share/aprl_navigation/config", glob("config/*.yaml")),
        ("share/aprl_navigation/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="APRL",
    maintainer_email="aprl@example.invalid",
    license="MIT",
    description="APRL Nav2 bringup and velocity arbitration",
    entry_points={"console_scripts": ["command_mux = aprl_navigation.command_mux:main"]},
)
