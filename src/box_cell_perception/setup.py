from glob import glob

from setuptools import find_packages, setup

package_name = "box_cell_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jr",
    maintainer_email="vlassom0@gmail.com",
    description="camera_node, label_reader(QR), pose_resolver(호모그래피)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node = box_cell_perception.camera_node:main",
            "label_reader = box_cell_perception.label_reader:main",
            "pose_resolver = box_cell_perception.pose_resolver:main",
            "stack_check = box_cell_perception.stack_check:main",
        ],
    },
)
