from setuptools import setup

package_name = 'visitor_agent'

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
    description='Visitor Agent Node for Campus Virtual Tour',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visitor_agent_node = visitor_agent.visitor_agent:main',
        ],
    },
)
