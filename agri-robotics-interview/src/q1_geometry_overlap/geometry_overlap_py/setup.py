from setuptools import find_packages, setup

package_name = 'geometry_overlap_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/q1_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Lambert',
    maintainer_email='jason.lambert@sabantoag.com',
    description=(
        'Interview exercise: compare/overlap geometries reported by an '
        'internal geometry provider node.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geometry_provider_node = geometry_overlap_py.geometry_provider_node:main',
            'overlap_checker_node = geometry_overlap_py.overlap_checker_node:main',
        ],
    },
)
