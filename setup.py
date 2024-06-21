from setuptools import find_packages, setup

package_name = 'theodolite_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/theodolite_pose.yaml']),
        ('share/' + package_name + '/config', ['config/icp_pose.yaml']),
        ('share/' + package_name + '/launch', ['launch/theodolite_pose.launch.py']),
        ('share/' + package_name + '/launch', ['launch/icp_pose.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Effie Daum',
    maintainer_email='efdau@ulaval.ca',
    description='A package for recording ground truth poses of a theodolite with the Warthog robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_node = theodolite_pose.ground_truth_three_dof:main',
            'icp_pose = theodolite_pose.icp_theodolite_pose:main'
        ],
    },
)