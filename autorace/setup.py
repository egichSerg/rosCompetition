from setuptools import find_packages, setup

package_name = 'autorace'
submodules = 'autorace/submodules'

setup(
    name=package_name,
    version='0.0.1a',
    packages=[package_name, submodules], # find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoy',
    maintainer_email='e.sergeev@g.nsu.ru',
    description='autorace package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector=autorace.object_detection:main',
            'corrector=autorace.lane_corrections:main',
            'pid=autorace.pid:main'
        ],
    },
)
