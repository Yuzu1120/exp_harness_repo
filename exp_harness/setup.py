import os
from glob import glob
from setuptools import setup

package_name = "exp_harness"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "test"), ["test/test.bash"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yuzuki Fujita",
    maintainer_email="fujifujiyuzunoki@icloud.com",
    description="実験自動化ハーネス本体（A/B切替・計測・レポート出力）",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "metric_pub_node = exp_harness.metric_pub_node:main",
            "experiment_server_node = exp_harness.experiment_server_node:main",
            "report_printer_node = exp_harness.report_printer_node:main",
        ],
    },
)
