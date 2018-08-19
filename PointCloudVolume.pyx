# distutils: language = c++
# cython: infer_types = True

import numpy as np
from PointCloudVolume cimport pointCloudVolume
cimport cython

@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing.
def calculateVolume( row ):
    arr = np.array( row , dtype=np.float64 )
    cdef double[::1] arr_view = arr

    N = len(row)/6
    cdef double vol = pointCloudVolume(N, &arr_view[0])
    return vol
