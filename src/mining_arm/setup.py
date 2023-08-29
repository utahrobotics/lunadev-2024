from setuptools import setup

package_name = 'mining_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyvesc'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='najman.husaini@utah.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mining_arm = mining_arm.mining_arm:main'
        ],
    },
)
