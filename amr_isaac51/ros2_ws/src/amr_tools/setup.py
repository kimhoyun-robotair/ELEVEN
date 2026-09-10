from glob import glob
from setuptools import find_packages, setup

setup(
    name="amr_tools",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/amr_tools"]),
        ("share/amr_tools", ["package.xml"]),
        ("share/amr_tools/launch", glob("launch/*.launch.py")),
        ("share/amr_tools/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AMR package maintainer",
    maintainer_email="maintainer@example.invalid",
    description="Teleoperation, stream monitoring and live smoke validation",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "teleop = amr_tools.teleop:main",
        "monitor = amr_tools.monitor:main",
        "smoke = amr_tools.smoke:main",
    ]},
)
