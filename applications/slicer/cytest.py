import numpy as np
import time

#interesting notes on performance here.
#in each case the task was to loop over a 600 x 600 array and add 2 to the value
#results were:
# pure python: 52ms
# pure java : 4ms
# python calling into numpy 800ms (!)
# python converted to c using cython with only basic optimization: 2.5ms
# python with bounds checks disabled 1.8ms
# python with bounds check and wrap around disabled: 0.9ms
# also disable nonecheck: 0.6ms


def addScalar(array,size,value):
	for i in range(size):
		for j in range(size):
			array[i][j] += value;
			
def test():
	SIZE = 600;
	#a = np.zeros((SIZE,SIZE));
	a = [];
	for i in range(SIZE):
		a.append([]);
		for j in range(SIZE):
			a[i].append(0);
	#print a;
	q = time.clock();
	addScalar(a,SIZE,2);

	print "%0.5f sec" % ( time.clock() - q );
	#print a;
	
if __name__=='__main__':
	test();

	
