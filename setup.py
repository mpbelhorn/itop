from distutils.core import setup

setup(
    name='itop',
    version='0.2.0',
    author='M.P. Belhorn',
    author_email='matt.belhorn@gmail.com',
    packages=[
        'itop',
        'itop.beam',
        'itop.math',
        'itop.motioncontrol',
        'itop.utilities'],
    scripts=[],
    url='https://github.com/emmpbee/ucbelle-optics',
    license='MIT',
    description='A package to test iTOP mirrors and prisms at UC.',
    long_description=open('README.txt').read(),
    install_requires=[
        "pyserial >= 2.6",
        "numpy >= 1.7.0",
    ],
)
