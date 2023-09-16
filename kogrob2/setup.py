import os
from glob import glob
from setuptools import setup

package_name = 'kogrob2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
        (os.path.join("share", package_name), glob("rviz/*config.rviz")),
        (os.path.join("share", package_name), glob("params/*params.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='martin@todo.todo',
    description='kogrob2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            "features_extractor_node = kogrob2.features_extractor:main",
            "class_interaction_node = kogrob2.class_interaction:main",
        ],
    },
)
