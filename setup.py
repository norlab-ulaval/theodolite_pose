from setuptools import find_packages, setup

package_name = 'theodolite_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='effie',
    maintainer_email='efdau@ulaval.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_theodolite = theodolite_pose.ground_truth_theodolite:main',
            'calibration = theodolite_pose.calibration:main'
        ],
    },
)
