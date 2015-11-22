from distutils.core import setup, Extension
from catkin_pkg.python_setup import generate_distutils_setup

ffpModule = Extension('ffp', 
                    sources = ['src/ffp.c'],
                    libraries = ['dl'],
                    extra_link_args = ['-rdynamic']
                   )

setup_args = generate_distutils_setup(ext_modules = [ffpModule])
setup(**setup_args)

