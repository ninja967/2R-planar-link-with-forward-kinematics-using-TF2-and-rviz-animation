import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'twoplanar_arm'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.launch')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'cad'), glob('cad/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sarat',
    maintainer_email='sarat@email.com',
    description='Two-link planar inspection arm with forward kinematics, TF2, and RViz animation',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sine_wave_joint_publisher = twoplanar_arm.sine_wave_joint_publisher:main',
            'forward_kinematics_verifier = twoplanar_arm.forward_kinematics_verifier:main',
            'test_fk_standalone = twoplanar_arm.test_fk_standalone:main',
        ],
    },
)
