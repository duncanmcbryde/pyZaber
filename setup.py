# Copyright 2013 Knowledge Economy Developments Ltd
#
# Henry Gomersall
# heng@kedevelopments.co.uk
#

from distutils.core import setup
from distutils.util import get_platform

version = '0.1'

setup_args = {
        'name': 'pyZaber',
        'version': version,
        'author': 'Henry Gomersall',
        'author_email': 'heng@kedevelopments.co.uk',
        'description': 'A python interface to the zaber linear slides',
        'classifiers': [
            'Programming Language :: Python',
            'Programming Language :: Python :: 3',
            'Development Status :: 3 - Alpha',
            'Operating System :: OS Independent',
            'Intended Audience :: Developers',
            'Intended Audience :: Science/Research',
            'Topic :: Scientific/Engineering',
            ],
        'packages':['pyzaber'],
  }

if __name__ == '__main__':
    setup(**setup_args)
