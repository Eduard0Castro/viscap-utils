from setuptools import find_packages, setup

package_name = 'viscap_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardo',
    maintainer_email='americogomes1@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "multiranger_test = viscap_utils.examples.multiranger_test:main",
            "swarm_test = viscap_utils.examples.swarm_test:main",
            "ai_deck = viscap_utils.examples.ai_deck_example:main",
        ],
    },
)
