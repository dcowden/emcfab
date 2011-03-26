import sys,time
import numpy as np

def rm(d,keys):
	for k in keys:
		if d.has_key(k): del d[k];

"given a starting point as a tuple, find the end point of the segement"
def findSegment(array,(x0,y0) ):
	"""
		attempts to find a string of pixels that makes the longest possible
		line. 
		The assumption is that the pixels were created using bresenham's line
		alogrithm, interpolating a line.

		integer math is assumed.
		
		assumes the starting point is set, and that it is valid in the array
	"""
	dy  =0;
	dx = 0;
	x = x0;
	y = y0;
	slopeAccumulator = [0,0 ];
	lastSlopes = None;	
	currentDir = None;

	dirs =  { 'n': (0,1), 's' : (0,-1) , 'e' : (1,0), 'w' : (-1,0), 'ne':(1,1) ,'se':(-1,1),'sw':(-1,-1 ),'nw':(1,-1) };
	
	assert array[x,y] == 1,'Initial Point must be set';
	
	while True:
		#search for a neighboring pixel
		#d is index offsets from current location
		foundNext = False;
		for (k,v) in dirs.iteritems(): #after two loop iterations there will only be two choices left
			nx = x + v[0];
			ny = y + v[1];
			t = array[(nx,ny)];
			if t == 1:
				dx = v[0];
				dy = v[1];
			
				foundNext = True;
				#print "Next Point:",(nx,ny);
				#is there a better way to do this? ideally we dont have to test these every time
				#but this is ok to test the algo in general
				if len(dirs) >2:					
					if k == 'e':
						rm(dirs,['w','n','s','nw','sw'] );				
					elif k == 'w':
						rm(dirs,['e','n','s','ne','se']);				
					elif k == 's':
						rm(dirs,['n','e','nw','ne']);				
					elif k == 'n':
						rm(dirs,['s','e','w','sw','se']);
					elif k == 'ne':
						rm(dirs,['s','w','sw','se','nw']);				
					elif k == 'nw':
						rm(dirs,['s','e','sw','se','ne']);
					elif k == 'sw':
						rm(dirs,['n','e','nw','ne','se']);
					elif k == 'se':
						rm(dirs,['n','w','ne','nw','sw']);						
					#print "Dirs Left:",dirs;
				
				#decrement slope accumulators
				if lastSlopes != None:

					if lastSlopes[0] < -2:
						print "Next Pixel was out of bounds for x"
						return (x,y);
					if lastSlopes[1] < -2:
						print "next Pixel was out of bounds for y"
						return (x,y);
						
					lastSlopes[0] -= dx;
					lastSlopes[1] -= dy;
						
				if currentDir != None and currentDir != k:
					#switch direction. store accumulated slopes.
					#print "Switching Direction"
					#check accumulators to ensure that we switched about the right time
					if lastSlopes != None:
						if lastSlopes[0] > 2:
							print "Next pixel out of bounds for x: unexpected move"
							return (x,y)
						if lastSlopes[1] > 2:
							print "Next pixel out of bounds for y: unexpected move"
							return (x,y)

					lastSlopes = slopeAccumulator;
					slopeAccumulator = [ 0,  0 ];
				
				slopeAccumulator[0] += dx;
				slopeAccumulator[1] += dy;	
				currentDir = k;
				x = nx;
				y = ny;
				break;
		if not foundNext:
			#print "No Neigboring Pixel. Stopping."
			return (x,y);
				
"""
	Performance Note: The c-extension version of this code runs 10x faster, even without
	disabling bounds checking.  this version takes 1ms to plot a 200 point line, 
	the c version plots the same line in 0.1 ms.

"""
def piecewise_bresenham_line(array,(x0,y0),(x2,y2),minSegmentLength=0):
    """
		Brensenham line algorithm, but slightly modified:
		  draws on the provided numpy array
		  avoids re-drawing pixels already drawn
		  returns a list of line segments that were drawn, inclusive of ends
		  
		minSegmentLength is the minimum length of a segment that will be generated
		Typically this is used to eliminate single pixels length segments
  """
    steep = 0
    writing=0
    lines = []
    
    x,y = x0,y0
    segX = 0;
    segY = 0;
    dx = abs(x2 - x)
    if (x2 - x) > 0: sx = 1
    else: sx = -1
    dy = abs(y2 - y)
    if (y2 - y) > 0: sy = 1
    else: sy = -1
    if dy > dx:
        steep = 1
        x,y = y,x
        dx,dy = dy,dx
        sx,sy = sy,sx
    d = (2 * dy) - dx
	
    #main loop more complex than normal since we check the underlying array
    for i in range(0,dx):

	if steep:
		c0 = y; c1=x; 
	else:
		c0 = x; c1=y; 
		
	if array[c0,c1] > 0:
		if writing:
			#end segment if not too short
			if minSegmentLength > 1:
				length = max( abs(segX - c0), abs(segY - c1 ) );
				if length >= minSegmentLength:
					lines.append( [ (segX, segY),(lastc0,lastc1) ] );
			else:
				lines.append( [ (segX, segY),(lastc0,lastc1) ] );
			writing = 0;
	else:
		array[c0,c1] = 1;
		if not writing:
			#start segment
			writing = 1;
			segX = c0;
			segY  = c1;

	lastc0 = c0;
	lastc1 = c1;			
        while d >= 0:
            y = y + sy
            d = d - (2 * dx)
        x = x + sx
        d = d + (2 * dy)
    
    #finish up
    if writing:
	#end segment
	array[x2,y2] = 1;
	lines.append( [(segX,segY),(x2,y2)] );
    
    return lines;
	

def testPieceWiseBresenhamPerformance():
	"test performance"
	q = time.clock();
	a  = np.zeros((1000,1000),dtype=np.uint8 );
	LOOPS=200;
	for i in range(0,LOOPS):
		lines  = piecewise_bresenham_line(a,(i,0),(i+i,i) );
	print "%d iters, %0.5f sec per iter" %  (LOOPS, (time.clock() - q )/LOOPS);
	
def testPieceWiseBresenham():
	"""
		Tests code that does piecewise bresenham algorithm.
		
	"""
	
	# a stripe down the middle
	a  = np.zeros((12,12),dtype=np.uint8 );
	a[:,5:7] =1;
	#print a;

	#draw a vertical line
	P1,P2 = (0,9),(0,10);
	lines = piecewise_bresenham_line(a,P1,P2);
	assert len(lines) == 1, "One Line"
	assert lines[0] == [ P1,P2 ], "Should be simple line"

	#x=y line.
	P1,P2=(0,0),(9,9);
	lines = piecewise_bresenham_line(a,P1,P2 );
	assert len(lines) == 2, "Should be two segments"
	assert lines[0][0] == P1, "First point should be (0,0)"
	assert lines[0][1] == ( 4,4 ), "Second point in first line should be (4,4)"
	
	#horizontal line. corner overlaps with previous
	lines = piecewise_bresenham_line(a,( 0,0 ),(5,0 ) );
	assert lines[0] == [ (1,0),(5,0) ], "First Line should be (1,0) --> (5,0). 0,0 was already covered "
	assert len(lines) == 1, "Should be one line segment"
	assert lines[0][0][0] == 1,"First Point should start at x=1"

	#vertical line that creates single unfilled pixel
	lines =  piecewise_bresenham_line(a,( 2,0 ),(2,6 ),2 );
	assert lines[0] == [ (2,3 ), (2,4 )], "Line should skip (2,0) is already covered, (2,1) \
 	   is open but short segment, (2,2) is covered, line goes 2,3 - 2,4"
	assert len(lines) == 1, "Should be one line"

	#line that produces no drawing
	P1,P2 = (2,2),(4,4);
	lines = piecewise_bresenham_line(a,P1,P2);
	assert len(lines) == 0, "Should be no entries in this line"
	
def testCorrectSegments():
	"test cases for finding segments,where the segment is valid"
	startPoint = (1,1);
	endPoints = [ ( 2,2 ),(6,6), (1,9), (9,1) ,(3,8), (8,3), (2,5),(9,7),(2,8),(9,8),(8,9) ];
	#endPoints = [ (9,8) ];
	for p in endPoints:
		print "Test Case %s --> %s " % ( str(startPoint), str(p) ),;
		a = np.zeros((12,12),dtype=np.uint8 );
		line(a,startPoint,p );
		q = findSegment ( a, startPoint );
		if q != p:
			print "Did not find end point" + str(p);
			print a;
		else:
			print "Passed."
			

if __name__=='__main__':
	#testCorrectSegments();
	print "Running Tests....."
	testPieceWiseBresenham();
	print "Testing Performance..."
	testPieceWiseBresenhamPerformance();
	
	