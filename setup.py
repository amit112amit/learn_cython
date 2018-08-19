from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

extensions= [
    Extension('pointcloudvolume', ['PointCloudVolume.pyx'],
              include_dirs=['/usr/include/eigen3'],
              libraries=['CGAL','CGAL_Core','gmp']),
]

setup(
    name="Point Cloud Volume",
    ext_modules=cythonize(extensions),
)
