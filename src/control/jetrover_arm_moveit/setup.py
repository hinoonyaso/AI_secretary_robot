from pathlib import Path
from setuptools import setup

package_name = 'jetrover_arm_moveit'


def package_files(directory: str):
    base = Path(directory)
    files = []
    for path in base.rglob('*'):
        if path.is_file():
            install_dir = str(Path('share') / package_name / path.parent)
            files.append((install_dir, [str(path)]))
    return files

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/moveit_demo.launch.py']),
        (
            'share/' + package_name + '/config',
            [
                'config/jetrover_arm.srdf',
                'config/kinematics.yaml',
                'config/ompl_planning.yaml',
                'config/joint_limits.yaml',
                'config/moveit_controllers.yaml',
                'config/servo_calibration.yaml',
            ],
        ),
    ] + package_files('urdf') + package_files('meshes'),
    install_requires=[],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='MoveIt2 integration package for JetRover arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_moveit_commander = jetrover_arm_moveit.arm_moveit_commander:main',
            'capture_servo_centers = jetrover_arm_moveit.capture_servo_centers:main',
        ],
    },
)
