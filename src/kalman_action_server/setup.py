from setuptools import setup

package_name = 'kalman_action_server'

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
    maintainer='benjamin',
    maintainer_email='benjaminhoej@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = kalman_action_server.main:main',
            'test = kalman_action_server.test:main'
        ],
    },
)
