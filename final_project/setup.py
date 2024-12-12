from setuptools import find_packages, setup

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', [
            'resource/templates/index.html'
        ]),
        ('share/' + package_name + '/static', [
            'resource/static/style.css'
        ]),
        ('share/' + package_name + '/audio', [
            'resource/audio/q1.mp3',
            'resource/audio/q2.mp3',
            'resource/audio/q3.mp3',
            'resource/audio/trimmed_q4.mp3'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Final_Project',
    maintainer_email='sk2866@cornell.edu',
    description='Final_Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_architecture = final_project.robot_control_architecture:main'
        ],
    },
)
