from setuptools import setup

package_name = 'rb23_audio'

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
    maintainer='denis',
    maintainer_email='denis.mosconi@ifsp.edu.br',
    description='Nó de áudio do GuardROS para o RB23',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rb23_audio_node = rb23_audio.rb23_audio_node:main',
        ],
    },
)