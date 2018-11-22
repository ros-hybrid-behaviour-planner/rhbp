from distutils.core import setup, Extension
from catkin_pkg.python_setup import generate_distutils_setup
import os
import sysconfig

ffModule = Extension('ff',
                     define_macros=[('PYTHON', '1')],
                     libraries=['m'],
                     sources=[
                                "main.c",
                                "memory.c",
                                "output.c",
                                "parse.c",
                                "expressions.c",
                                "inst_pre.c",
                                "inst_easy.c",
                                "inst_hard.c",
                                "inst_final.c",
                                "relax.c",
                                "search.c",
                                "scan-fct_pddl.tab.c",
                                "scan-ops_pddl.tab.c"
                               ]
                     )
setup_args = generate_distutils_setup(ext_modules = [ffModule])
setup(**setup_args)
#for name,value in distutils.sysconfig.get_config_vars().iteritems():
#    print("{0}={1}".format(name, value))
#print(sysconfig.get_paths())
#print(setup_args)
#with open("setup.cfg", "w") as config:
#    config.write("[install]\ninstall_lib={0}/lib".format(os.environ['CMAKE_INSTALL_PREFIX']))
