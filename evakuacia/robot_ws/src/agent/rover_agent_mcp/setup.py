from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_agent_mcp'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'FLEET_PROTOCOL.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='ROS 2 text-command agent with an MCP-style JSON-RPC tool server for rover control.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agent_text_node = rover_agent_mcp.agent_text_node:main',
            'rover_mcp_server = rover_agent_mcp.rover_mcp_server:main',
        ],
    },
)
