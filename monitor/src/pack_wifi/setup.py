from setuptools import setup

package_name = 'pack_wifi'

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
    maintainer='KianBehzad',
    maintainer_email='behzad.k@northeastern.edu',
    description='WiFi CSI data collector',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'csi_node = {package_name}.csi_node:main',
            f'heartbeat_node = {package_name}.heartbeat_node:main',
            f'dummy_receiver_node = {package_name}.dummy_receiver_node:main',
            f'csi_node_test = {package_name}.csi_node_test:main',
        ],
    },
)
