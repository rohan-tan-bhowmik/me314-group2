from setuptools import find_packages, setup
import glob

package_name = 'dollar_bill'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ac1@stanford.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pos_from_pixel = dollar_bill.posFromPixel:main',
            'image_to_pixel = dollar_bill.pixelFromImage:main',
            'move_to_point = dollar_bill.moveToPoint:main',
            'run = dollar_bill.run:main',
        ],
    },
)

