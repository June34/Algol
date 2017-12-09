#!/usr/bin/env python3

import sys
from setuptools import setup
from setuptools import find_packages

if sys.version_info[:3] < (3, 3):
    raise SystemExit("Algol needs Python 3.3.")

setup(
    name='Algol',
    version='0.1dev',
    description='RISC-V core',
    long_description=open('README.md').read(),
    author='Angel Terrones',
    author_email='angelterrones@gmail.com',
    license="MIT",
    platforms=['Any'],
    keywords='FPGA myHDL RISC-V',
    packages=find_packages()
)

# Local Variables:
# flycheck-flake8-maximum-line-length: 200
# flycheck-flake8rc: ".flake8rc"
# End:
