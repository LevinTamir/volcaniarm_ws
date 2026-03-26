from setuptools import setup, find_packages

package_name = 'volcaniarm_weed_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tamir',
    maintainer_email='tamir@example.com',
    description='Weed 3D Detector Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weed_detection_node = volcaniarm_weed_detector.weed_detection_node:main'
        ],
    },
)
