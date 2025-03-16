from setuptools import find_packages, setup
# 추가
from glob import glob

package_name = 'PJT_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 추가
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools',],
    zip_safe=True,
    maintainer='chichi',
    maintainer_email='chichi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place = PJT_server.pick_and_place:main',
            'seperator_with_yolo = PJT_server.seperator_with_yolo:main',
            'data_sending_server = PJT_server.data_sending_server:main',
        ],
    },
)
