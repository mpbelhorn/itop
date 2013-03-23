from distutils.core import setup

setup(
    name='itop',
    version='0.1.1',
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
    license='LICENSE.txt',
    description='A package to test iTOP mirrors and prisms at UC.',
    long_description=open('README.txt').read(),
    install_requires=[
        "pyserial >= 2.6",
        "numpy >= 1.7.0",
    ],
)
