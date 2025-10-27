from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ai_inference'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include model files
        (os.path.join('share', package_name, 'models'), glob('models/*.tflite')),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pb')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='QIRP Developer',
    maintainer_email='developer@qirp.local',
    description='AI inference package for QIRP device with TensorFlow Lite models',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = ai_inference.yolo_detector:main',
        ],
    },
)
