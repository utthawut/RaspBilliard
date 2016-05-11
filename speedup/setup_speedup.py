from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy   # << New line

ext_modules = [Extension("speedup", ["SpeedUp.pyx"])]

setup(
    name = 'test',
    cmdclass = {'build_ext': build_ext},
    include_dirs = [numpy.get_include()], # << New line
    ext_modules = ext_modules
)
