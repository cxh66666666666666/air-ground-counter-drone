from glob import glob

from setuptools import setup

package_name = 'agc_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rl.yaml', 'config/w7.yaml']),
        ('share/' + package_name + '/launch', ['launch/rl_inference.launch.py']),
        ('share/' + package_name + '/policies', glob('policies/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGC Dev',
    maintainer_email='agc@example.com',
    description='AGC RL 平台（W5-b 地面最小版 + W7-a 最小平台）',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'ground_policy_rl = agc_rl.inference_node:main',
            # W7-b P2 臂 B：高层（拦截点）决策节点；低层 = ground_baseline（Navfn + 自研 LQR）
            'agc_rl_high_level = agc_rl.high_level_node:main',
        ],
    },
)
