from setuptools import setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoonseo',
    maintainer_email='peanutyoun@naver.com',
    description='path planning package',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning_node = path_planning.path_planning_node:main',
        ],
    },
)

