from distutils.core import setup, Extension
from catkin_pkg.python_setup import generate_distutils_setup

ffModule = Extension('ff',
                     define_macros = [('PYTHON', '1')],
                     libraries = ['m'],
                     sources = [
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
setup_args = generate_distutils_setup()
setup_args["ext_modules"] = [ffModule]
#print setup_args
setup(**setup_args)
