from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy as np

setup(
	name = 'brushtest',
	ext_modules=[
	   Extension('brushes_c',['brushes.pyx'])
	   ],
	   cmdclass = {'build_ext':build_ext},
	   include_dirs = [np.get_include()]
 )