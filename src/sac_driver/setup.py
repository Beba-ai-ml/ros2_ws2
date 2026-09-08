from setuptools import setup, find_packages
from glob import glob

package_name = "sac_driver"


def _data_files_if_any(dest, pattern):
    """Return [(dest, files)] when the glob matches, otherwise []."""
    files = sorted(glob(pattern))
    return [(dest, files)] if files else []

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        # Model checkpoints are installed into the package share dir so that
        # relative paths such as "weights/foo.pth" resolve on any machine
        # (see _resolve_path in sac_driver/sac_driver_node.py).
        # The entry is dropped when the glob is empty so an install without
        # checkpoints still succeeds.
    ]
    + _data_files_if_any(f"share/{package_name}/weights", "weights/*.pth"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LLM_migracja",
    maintainer_email="noreply@example.com",
    description="SAC policy driver for F1TENTH integration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sac_driver_node = sac_driver.sac_driver_node:main",
        ],
    },
)
