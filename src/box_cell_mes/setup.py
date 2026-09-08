from glob import glob

from setuptools import find_packages, setup

package_name = "box_cell_mes"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/static", glob("static/*")),
        ("share/" + package_name, ["box_cell_mes/seed_items.json"]),
    ],
    package_data={"box_cell_mes": ["seed_items.json"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jr",
    maintainer_email="vlassom0@gmail.com",
    description="MES 서버(FastAPI + SQLite)와 대시보드",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mes_server = box_cell_mes.server:main",
        ],
    },
)
