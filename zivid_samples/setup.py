import os
from glob import glob
from setuptools import setup


package_name = 'zivid_samples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        # Same with the RViz configuration file.
        (os.path.join('share', package_name, "config"), glob('config/*')),
        # And the ply files.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lars',
    maintainer_email='lars.tingelstad@ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_request = zivid_samples.capture_request:main',
            'sample_capture = zivid_samples.sample_capture:main',
            'sample_capture_2 = zivid_samples.sample_capture_2:main',
            'sample_capture_3 = zivid_samples.sample_capture_3:main'
        ],
    },
)
