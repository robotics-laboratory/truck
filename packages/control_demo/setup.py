from glob import glob

from setuptools import setup

package_name = 'control_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='simagin.mail@yandex.ru',
    description='Publish clicked path in foxglove',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "control_demo = control_demo.node:main",
        ],
    },
)
