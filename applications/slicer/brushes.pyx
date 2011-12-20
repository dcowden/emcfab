import numpy as np
import time,cProfile,pstats
cimport numpy as np
cimport cython
DTYPE=np.int
ctypedef np.int_t DTYPE_t

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
def paint3Brush(np.ndarray[DTYPE_t,ndim=2] array not None,int x=0,int y=0):
	array[x-1,y] = 0;
	array[x,y] = 0;
	array[x+1,y] = 0;
	array[x,y+1] = 0;
	array[x,y-1] = 0;

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
def paint7Brush(np.ndarray[DTYPE_t,ndim=2] array not None,int x=0,int y=0,int value=0):
	"""
		Paint the values, but first check each one to ensure it is not already painted.
	"""
	drawn = 1;
	#test values for existing painting
	if array[x,y+2] > 7 or array[x-1,y+1] > 7 or array[x,y+1] > 7 or array[x+1,y+1] > 7 or array[x-2,y] > 7 or array[x-1,y] > 7:
		drawn = 0;
		print "Found existing!"
	if array[x+1,y] > 7 or array[x+2,y] > 7 or array[x-1,y-1] > 7 or array[x,y-1] > 7 or array[x+1,y-1] > 7 or array[x,y-2] > 7:
		drawn = 0;
		print "Found existing!"
	if drawn == 1:
		#row 1
		array[x,y+2] = value;

		#row 2
		array[x-1,y+1] = value;
		array[x,y+1] = value;
		array[x+1,y+1] = value;

		#row3
		array[x-2,y] = value;
		array[x-1,y] = value;
		array[x,y] = 8; #center value is special
		array[x+1,y] = value;	
		array[x+2,y] = value;
		
		#row4
		array[x-1,y-1] = value;
		array[x,y-1] = value;
		array[x+1,y-1] = value;

		#row5
		array[x,y-2] = value;
	return drawn

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)	
def doWrites(np.ndarray[DTYPE_t,ndim=2] a not None):
	
	q = time.clock();
	drawn =0;
	
	#this is a diagonal
	for i in range(10,4000,2):
		drawn = drawn + paint7Brush(a,i,i,1);
	print "Drew %d pixels" % (drawn)
	print "%0.3f elapsed" % ( time.clock() - q )
	
	drawn=0
	q = time.clock();
	#ok now draw a horizontal. we should draw less than all
	for i in range(20,220,3):
		drawn = drawn+ paint7Brush(a,100,i,1);
	print "Drew another %d pix" % ( drawn )
	print "%0.3f elapsed" % ( time.clock() - q )


	

		