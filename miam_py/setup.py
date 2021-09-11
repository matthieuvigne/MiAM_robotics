#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup


setup(
    name='miam_py',
    version='0.1.2',
    description='MiAM robotics log processing',
    long_description=open('README.md').read(),
    packages=['miam_py'],
    package_dir={'': 'src'},
    scripts=["scripts/miam_plot",
             "scripts/miam_merge",
             "scripts/miam_analyse_tracking",],
    data_files=[('images', ['images/table.png'])],
    zip_safe=False)
