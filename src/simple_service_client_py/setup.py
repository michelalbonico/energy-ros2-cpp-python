from setuptools import find_packages, setup

package_name = 'simple_service_client_py'

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
    description='Python client server tutorial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = simple_service_client_py.service_member_function:main',
            'client = simple_service_client_py.client_member_function:main',
        ],
    },
)
