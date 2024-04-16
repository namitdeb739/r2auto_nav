from setuptools import setup

package_name = 'auto_nav'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nus",
    maintainer_email="nus@todo.todo",
    description="This package provides functionalities for autonomous navigation including control, line following, payload handling, and exploration.",
    license="NIL",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control = auto_nav.control:main",
            "line = auto_nav.lineFollower:main",
            "payload = auto_nav.payload:main",
            "explore = auto_nav.explore:main",
            "r2auto_nav = auto_nav.r2files.r2auto_nav:main",
        ],
    },
)
