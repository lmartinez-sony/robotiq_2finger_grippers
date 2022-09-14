from setuptools import setup

package_name = 'robotiq_2f_gripper_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevinzhang',
    maintainer_email='kevinleezhang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'robotiq_2f_action_server = robotiq_2f_gripper_control.robotiq_2f_action_server:main',
    ],
    },
)
