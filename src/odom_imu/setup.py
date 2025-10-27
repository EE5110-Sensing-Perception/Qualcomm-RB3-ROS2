import os
from glob import glob
from setuptools import setup
from setuptools.command.install import install

package_name = 'odom_imu'

class InstallWithLib(install):
    def run(self):
        super().run()
        # Move scripts from bin to lib/package_name
        import shutil
        bin_dir = os.path.join(self.install_base, 'bin')
        lib_dir = os.path.join(self.install_base, 'lib', package_name)
        os.makedirs(lib_dir, exist_ok=True)
        if os.path.exists(bin_dir):
            for file in os.listdir(bin_dir):
                src = os.path.join(bin_dir, file)
                dst = os.path.join(lib_dir, file)
                if os.path.isfile(src):
                    shutil.copy2(src, dst)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/imu_ekf.yaml']),
        ('share/' + package_name + '/launch', ['launch/imu_ekf_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='IMU-only EKF with ZUPT and tilt correction',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_ekf_node = odom_imu.imu_ekf_node:main',
        ],
    },
    cmdclass={
        'install': InstallWithLib,
    },
)
