from setuptools import find_packages, setup

package_name = 'amr_plc_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed abdo',
    maintainer_email='61me38@gmail.com',
    description='Bridge between the fleet manager and the GTP station PLC (Modbus TCP).',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'plc_bridge = amr_plc_bridge.plc_bridge_node:main',
        ],
    },
)
