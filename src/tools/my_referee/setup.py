from setuptools import find_packages, setup
from glob import glob

package_name = 'my_referee'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/audio', glob(package_name + '/assets/audio/*.wav')),
        ('share/' + package_name + '/audio', glob(package_name + '/assets/audio/*.mp3')),
        ('share/' + package_name + '/audio', glob(package_name + '/assets/audio/music/*.wav')),
        ('share/' + package_name + '/audio', glob(package_name + '/assets/audio/music/*.mp3')),
        ('share/' + package_name + '/images', glob(package_name + '/assets/images/*.jpg')),
        ('share/' + package_name + '/images', glob(package_name + '/assets/images/*.png')),
        ('share/' + package_name + '/images', glob(package_name + '/assets/images/*.gif'))
    ],
    install_requires=['setuptools','pyside6', 'rclpy','robots_msgs'],
    zip_safe=True,
    maintainer='lehan',
    maintainer_email='150041695+gtx6901@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'my_referee_node = my_referee.main:main', 
        ],
    },
)
