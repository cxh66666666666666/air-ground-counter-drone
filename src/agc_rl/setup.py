from setuptools import setup

package_name = 'agc_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGC Dev',
    maintainer_email='agc@example.com',
    description='AGC RL 平台（W7）',
    license='Proprietary',
    entry_points={'console_scripts': []},
)