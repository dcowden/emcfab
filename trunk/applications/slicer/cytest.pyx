import numpy as np
import time
cimport numpy as np
cimport cython
DTYPE=np.int
ctypedef np.int_t DTYPE_t

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
def addScalar(np.ndarray[DTYPE_t,ndim=2] array not None,int value):
#cdef void addScalar(np.ndarray[DTYPE_t,ndim=2] array,int value):
	cdef int imax = array.shape[0]
	cdef int jmax = array.shape[1]
	cdef int i,j

	for i in range(imax):
		for j in range(jmax):
			array[i,j] += value;

		
def test():
	SIZE = 600;
	cdef np.ndarray[DTYPE_t,ndim=2] a
	a = np.zeros((SIZE,SIZE),dtype=np.int);
	
	
	#print a;
	q = time.clock();
	addScalar(a,2);

	#print a;
	print "%0.5f sec" % ( time.clock() - q );
	
if __name__=='__main__':
	test();


	
