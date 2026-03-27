from setuptools import setup
import os
from glob import glob

package_name = 'vcu_test'

# https://answers.ros.org/question/397319/how-to-copy-folders-with-subfolders-to-package-installation-path/


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files([
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        #(os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        #(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ], ['launch/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laurin',
    maintainer_email='laurin.hesslich@elbflorace.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         'efr15_roslog_handler = efr_launch.efr15_roslog_handler:main',
    #         'efr16_roslog_handler = efr_launch.efr16_roslog_handler:main',
    #     ],
    # },
)