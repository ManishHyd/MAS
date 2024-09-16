from setuptools import setup

package_name = 'bi_agent'

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
    description='BI Agent for Campus Virtual Tour',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bi_agent_node = bi_agent.bi_agent:main',
        ],
    },
)

