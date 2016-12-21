from setuptools import setup
from lindypy import __version__

setup(
    version = __version__,
    description = "Linda Tuple Spaces for Python",
    author = "Georg Bauer",
    author_email = "gb@rfc1437.de",
    url = "http://bitbucket.org/rfc1437/lindypy/",
    name='lindypy', 
    long_description=open("README").read(),
    license='MIT/X',
    platforms=['BSD','Linux','MacOS X'],
    packages=['lindypy'],
    scripts=['example_script.py'],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: BSD License",
        "Operating System :: POSIX",
        "Programming Language :: Python",
    ],
    test_suite="lindypy.tests",
)
