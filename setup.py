from setuptools import setup
import os
from glob import glob

package_name = "project2"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        ("share/" + package_name + "/data", glob("data/*.txt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="crae",
    maintainer_email="craeandrew@ufl.edu",
    description="Auto Vehicle Proj",
    license="N/A",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
        'VehControl = project2.VehControl:main',
        ],
    },
)
