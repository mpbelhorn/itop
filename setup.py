try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

setup(
    name='itop',
    version='0.6.0',
    author='M.P. Belhorn et al.',
    author_email='matt.belhorn@gmail.com',
    maintainer='M.P. Belhorn',
    maintainter_email='matt.belhorn@gmail.com',
    packages=[
        'itop',
        'itop.analyis',
        'itop.beam',
        'itop.math',
        'itop.motioncontrol',
        'itop.photodiode',
        'itop.raytrace',
        'itop.utilities',
        ],
    scripts=[
        'bin/scan-mirror',
        'bin/measure-tilt',
        ],
    url='https://github.com/emmpbee/itop',
    license='MIT',
    description='Suite for testing iTOP mirrors and prisms at the University of Cincinnati.',
    long_description=open('README.txt').read(),
    include_package_data=True,
    install_requires=[
        "pyserial >= 2.6",
        "numpy >= 1.7.0",
        "matplotlib >= 1.3.1",
    ],
)
