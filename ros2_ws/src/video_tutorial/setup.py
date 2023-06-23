from setuptools import setup

package_name = 'video_tutorial'

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
    maintainer='your_name',
    maintainer_email='your_name@todo.todo',
    description='Example of webcam video publisher and subscriber nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = video_tutorial.video_publisher:main',
            'video_subscriber = video_tutorial.video_subscriber:main',
        ],
    },
)
