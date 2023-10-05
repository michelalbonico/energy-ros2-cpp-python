from setuptools import find_packages, setup

package_name = 'simple_pub_sub_py'

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
    maintainer='michel',
    maintainer_email='bsd.albonico@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = simple_pub_sub_py.minimal_publisher:main',
            'listener = simple_pub_sub_py.minimal_subscriber:main'
        ],
    },
)
