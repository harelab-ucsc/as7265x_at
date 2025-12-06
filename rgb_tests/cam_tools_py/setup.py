from setuptools import setup

package_name = 'cam_tools_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi4',
    maintainer_email='you@example.com',
    description='Camera tools for libcamera + PPS stamping and splitting',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stamp_and_split = cam_tools_py.stamp_and_split:main',
            'pps_stamp_and_split = cam_tools_py.pps_stamp_and_split:main',
        ],
    },
)
