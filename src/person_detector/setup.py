from setuptools import setup, find_packages

package_name = 'person_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                        'torch',
                        'torchvision',
                        'opencv-python',
                        'numpy'],
    zip_safe=True,
    maintainer='emil',
    maintainer_email='emil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'person_detector = person_detector.person_detector_node:main',
            'person_detector_head_test = person_detector.person_detector_node:main',
            'person_detector_test = person_detector.person_detector_test_node:main',
            'person_detector_test2 = person_detector.person_detector_test2_node:main'
        ],
    },
)
