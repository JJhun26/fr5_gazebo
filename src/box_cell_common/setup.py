from setuptools import find_packages, setup

package_name = "box_cell_common"

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
    description="셀 제원을 코드 쪽에서 읽는 공용 모듈",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
