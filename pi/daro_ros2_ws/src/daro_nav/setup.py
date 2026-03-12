from setuptools import find_packages, setup
import os
from glob import glob

package_name = "daro_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # .gitkeep ensures the directory exists in the install tree.
        # glob picks up any saved maps (.yaml + .pgm) checked into the repo.
        (os.path.join("share", package_name, "maps"),
            ["maps/.gitkeep"] + glob("maps/*.yaml") + glob("maps/*.pgm")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="pi@todo.todo",
    description="DARO Nav2 navigation stack",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": []},
)
