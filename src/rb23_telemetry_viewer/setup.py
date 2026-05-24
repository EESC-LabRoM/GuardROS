from setuptools import setup

package_name = "rb23_telemetry_viewer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="denis",
    maintainer_email="denis.mosconi@ifsp.edu.br",
    description="Viewer textual de telemetria para o GuardROS / RollerBot 23.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rb23_telemetry_viewer_node = rb23_telemetry_viewer.rb23_telemetry_viewer_node:main",
        ],
    },
)