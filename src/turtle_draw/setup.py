from setuptools import find_packages, setup

package_name = 'turtle_draw'

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
    maintainer='breno',
    maintainer_email='breno.santos.ismart@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_draw = turtle_draw.test:main",
            "publisher = turtle_draw.publisher:main",
            "subscriber = turtle_draw.subscriber:main",
        ],
    },
)
