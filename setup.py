from setuptools import find_packages, setup

package_name = 'ti_es_gyroscope_package'

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
    maintainer='niko',
    maintainer_email='niko@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ti_es_gyroscope_node = ti_es_gyroscope_package.ti_es_gyroscope_node:main'
        ],
    },
)
