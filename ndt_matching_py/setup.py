from setuptools import setup

package_name = 'ndt_matching_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'laser_geometry'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Python NDT Matching Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ndt_node = ndt_matching_py.ndt_node:main',
            'initial_pose_from_cloud = ndt_matching_py.initial_pose_from_cloud:main',

        ],
    },
)
