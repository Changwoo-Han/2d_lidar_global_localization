from setuptools import find_packages, setup

package_name = 'icp_localizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy','open3d','transforms3d',],
    zip_safe=True,
    maintainer='croft_robot',
    maintainer_email='cwhan@croft-ai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_localization = icp_localizer.icp_localizer_node:main',
            'icp_initial_pose_estimator = icp_localizer.icp_initial_pose_estimator:main',
            'open3d = icp_localizer.open3d:main',
            'tf_and_icp = icp_localizer.tf_and_icp:main',
            'rotate_robot = icp_localizer.rotate_robot_node:main',

            

        ],
    },
)
