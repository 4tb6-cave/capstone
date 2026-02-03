from setuptools import find_packages, setup

package_name = 'berk'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),  # ✅ auto-detects 'berk' inside src/
    package_dir={'': 'src'},               # ✅ points to source directory
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', [  # ✅ install data files properly
#            'bno055_data.json',
#            'calibrationvalues.txt',
#            'poses.csv'
        ])
    ],
    install_requires=['numpy', 'open3d'],
    zip_safe=True,
    maintainer='Your Name',
    description='Point cloud saver node for ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pointcloud_saver = berk.pointcloud_saver:main'  # ✅ correct module path
        ],
    },
)
