from glob import glob

from setuptools import find_packages, setup

package_name = "box_cell_logic"

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
    description="task_manager 상태 기계, pallet_manager 적재 기록, mes_client",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_manager = box_cell_logic.task_manager:main",
            "pallet_manager = box_cell_logic.pallet_manager:main",
            "mes_client = box_cell_logic.mes_client:main",
        ],
    },
)
