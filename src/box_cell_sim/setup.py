from glob import glob


def _models():
    """models/ 아래를 디렉터리 구조 그대로 설치한다.

    라벨 텍스처(box_N.png)와 라벨 판 메시가 여기 있다. gz는 이것들을
    절대 경로로 읽으므로 share 아래 같은 모양으로 놓여야 한다.
    """
    import os

    out = []
    for root, _dirs, files in os.walk("models"):
        if files:
            out.append(
                ("share/" + package_name + "/" + root,
                 [os.path.join(root, f) for f in files])
            )
    return out

from setuptools import find_packages, setup

package_name = "box_cell_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        *_models(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jr",
    maintainer_email="vlassom0@gmail.com",
    description="Gazebo Harmonic 월드, 스폰, ros_gz 브리지",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "box_feeder = box_cell_sim.box_feeder:main",
        ],
    },
)
