from setuptools import find_packages, setup

package_name = 'rb23_video_viewer'

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
    maintainer='denis',
    maintainer_email='denis.mosconi@ifsp.edu.br',
    description='Visualizador de vídeo do GuardBot RollerBot 23',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rb23_video_viewer_node = rb23_video_viewer.rb23_video_viewer_node:main',
        ],
    },
)