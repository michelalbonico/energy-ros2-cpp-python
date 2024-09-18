from setuptools import find_packages, setup

package_name = 'simple_publisher_subscriber_py'

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
    maintainer='manuela',
    maintainer_email='manuelabechara@alunos.utfpr.edu.br',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = simple_publisher_subscriber_py.publisher_member_function:main',
            'listener = simple_publisher_subscriber_py.subscriber_member_function:main',
        ],
    },
)
