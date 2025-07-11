from setuptools import find_packages, setup

package_name = 'xbox_control'

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
    maintainer='evinia',
    maintainer_email='evinia.anasta@gmail.com',
    description='This package serves as a way to control the da Vinci tool using the Xbox 360 game controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_control_tool = xbox_control.xbox_controller_tool:main',
            'xbox_control_combo = xbox_control.xbox_controller_combo:main',
        ],
    },
)
