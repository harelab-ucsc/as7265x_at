from setuptools import find_packages, setup

package_name = 'as7265x_at'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mwmaster',
    maintainer_email='mwmaster@ucsc.edu',
    description='Simple driver for Sparkfun AS7265x Spectral Triad Spectrometer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'as7265x_at_node = as7265x_at.as7265x_at_node:main',
        ],
    },
)
