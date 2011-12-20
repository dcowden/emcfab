"""
	tests use of numpy to create 'brushes'
	for filling

"""

import numpy as np
import time,cProfile,pstats
"""
	returns a pair of numpy arrays, suitable for indexing an array
"""
def make3PixBrush((centerX,centerY)):
	return  ( np.array( [centerX, centerX, centerX, centerX-1, centerX +1 ]) ,
			  np.array( [centerY-1, centerY, centerY+1, centerY, centerY ]) );

def make7PixBrush((cX,cY)):
	"moving from left to right, and from top to bottom"
	x = np.array( [cY-1,cY,cY+1,cY-2,cY-1,cY,cY+1,cY+2] + [cY-3,cY-2,cY-1,cY,cY+1,cY+2,cY+3]*3 + [cY-2,cY-1,cY,cY+1,cY+2,cY-1,cY,cY+1] );
	y = np.array( [cX-3]*3 + [cX-2]*5 + [cX-1]*7 + [cX]*7 + [cX+1]*7 + [cX+2]*5 + [cX+3]*3 );
	return  ( x, y);

def doWrites():
	a = (np.ones(250000)).reshape(500,500);
	xi = range(10,400); 
	yi = range(10,400); 
	q = time.clock();
	for (x,y) in zip(xi,yi):
		a[make7PixBrush( (x,y ) )] = 0;
	print "%0.3f elapsed" % ( time.clock() - q )
def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
	p.print_stats(level);			
if __name__=='__main__':


	#set the pixels in the brush to zero
	#runProfiled('doWrites()',0.9 );
	doWrites();
	

		