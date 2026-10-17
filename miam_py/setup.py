#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup


setup(
    name='miam_py',
    version='1.0.0',
    python_requires='>=3.9',
    description='MiAM robotics log processing',
    long_description=open('README.md').read(),
    packages=find_packages('src'),
    package_dir={'': 'src'},
    entry_points={
        'console_scripts': [
            'miam_plot = miam_py.miam_plot:main',
            'miam_extract_text = miam_py.miam_extract_text:main',
            'miam_get_latest_log = miam_py.miam_get_latest_log:main',
            'miam_analyse_match = miam_py.miam_analyse_match:main',
        ],
    },
    package_data={'miam_py': ['images/table.png']},
    install_requires=["numpy", "matplotlib", "scipy", "tk", "h5py", "paramiko", "tqdm", "PyQt5"],
    zip_safe=False)
