from setuptools import setup

package_name = 'ci_agent'

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
    maintainer='manish',
    maintainer_email='manishvutkoori@gmail.com',
    description='CI Agent for Campus Virtual Tour',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ci_agent_node = ci_agent.ci_agent:main',
        ],
    },
)

