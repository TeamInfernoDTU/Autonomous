from setuptools import find_packages, setup

package_name = 'zed2_yolo_node'

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
    maintainer='de-snake',
    maintainer_email='work.shauryasinha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    entry_points={
    'console_scripts': [
        'zed2_yolo_node = zed2_yolo_node.main_node:main',
    ],
},
)
