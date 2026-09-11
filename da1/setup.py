from setuptools import find_packages, setup
import glob
import os


package_name = 'da1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name + '/zlac8015d', glob.glob("zlac8015d/*")),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanhpham',
    maintainer_email='thanhpham@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "motion = da1.motion:main",
            "m_to_m = da1.m_to_m:main",
            "move = da1.move:main",
            "detect = da1.detect:main",
            "algorithm = da1.algorithm:main"
        ],
    },
)
