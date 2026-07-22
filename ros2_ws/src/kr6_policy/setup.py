from setuptools import setup

package_name = 'kr6_policy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shahar Cohen',
    maintainer_email='cohenshahar17@gmail.com',
    description='KR6 VLA system: kr6_policy',
    license='MIT',
    entry_points={'console_scripts': ['policy_node = kr6_policy.policy_node:main']},
)
