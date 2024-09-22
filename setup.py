from setuptools import find_packages, setup

package_name = 'knowrob_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sascha Jongebloed',
    maintainer_email='jongebloed@uni-bremen.de',
    description='KnowRob ROS is a ROS wrapper for KnowRob 2.0.0',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
