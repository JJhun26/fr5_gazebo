from glob import glob

from setuptools import find_packages, setup

package_name = "box_cell_conveyor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jr",
    maintainer_email="vlassom0@gmail.com",
    description="conveyor_driver. 벨트 구동, 정지 센서 판정",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "conveyor_driver = box_cell_conveyor.conveyor_driver:main",
        ],
    },
)
