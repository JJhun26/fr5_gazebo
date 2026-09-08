from glob import glob

from setuptools import find_packages, setup

package_name = "box_cell_motion"

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
    description="motion_server(픽앤플레이스 액션)와 gripper_driver(석션 밸브)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_server = box_cell_motion.motion_server:main",
            "gripper_driver = box_cell_motion.gripper_driver:main",
            "scene_publisher = box_cell_motion.scene_publisher:main",
        ],
    },
)
