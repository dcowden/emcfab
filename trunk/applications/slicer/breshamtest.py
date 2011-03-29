import sys,time
import numpy as np


# test bits for the various connected pixels
#      432
#      501
#      678	
Q1 = 0b00000001
Q2 = 0b00000010
Q3 = 0b00000100
Q4 = 0b00001000
Q5=  0b00010000
Q6 = 0b00100000
Q7=  0b01000000
Q8 = 0b10000000

# combinations, computed ahead for performance
EAST = ( Q2 | Q1 | Q8 );
NORTH  = (Q4 | Q3 | Q2 );
WEST = ( Q4 | Q5 | Q6 );
SOUTH = ( Q6 | Q7 | Q8 );
NE = ( Q1 | Q2 | Q3 );
NW = ( Q3 | Q4 | Q5 );
SW = ( Q5 | Q6 | Q7 );
SE = ( Q7 | Q8 | Q1 );

VERTEX = 0b00000010
EDGE = 0b00000001

"""
	find a bresenham sequence.
	returns: the end point of the bresenham sequence starting at x0,y0 in the given array,
	or None if no sequnce can be found.
	
	array values with the second bit set are vertex-- we are done when we find one of these
	array values with the first bit set are an edge
	array values of zero are not part of the desired pattern.
	
	dirs is an 8-bit value used to track which directions in which the search will proceed.
	
	the code for the function is a lot more complex due to the functionality that 
    clears the underlying buffer, only if a line is found.
	
"""
def reverse_bresenham(array,x,y,dirs=0b11111111 ):

	#disable ending conditions if we are just starting.
	#one search forward is guaranteed to filter some directions
	if dirs != 0b11111111:		
		if array[x,y] == 0:
			return None;			
		if array[x,y] > VERTEX: #value is a vertex.	
			array[x,y] = 0;
			return (x,y);
		
	#else, we are on the starting vertex, or on an edge
	#search each candidate direction for a match, and, if found, continue that way
	
	#i cannot figure out how to avoid one if statement per direction.
	#each time we limit the choices to the appropriate quadrants. generally within just a couple of moves,
	#we will be down to running only two branches of this code
	if dirs &  Q1 :
		r = reverse_bresenham(array,x+1,y,dirs & EAST );
		if r != None: 
			array[x,y] = 0;
			return r;

	if dirs & Q2:
		r = reverse_bresenham(array,x+1,y+1,dirs & NE );
		if r != None: 
			array[x,y] = 0;
			return r;

	if dirs & Q3:
		r = reverse_bresenham(array,x,y+1,dirs & NORTH);
		if r != None: 
			array[x,y] = 0;
			return r;
			
	if dirs & Q4:
		r = reverse_bresenham(array,x-1,y+1,dirs & NW );
		if r != None: 
			array[x,y] = 0;
			return r;		
	if dirs & Q5:
		r = reverse_bresenham(array,x-1,y,dirs & WEST );
		if r != None: 
			array[x,y] = 0;
			return r;

	if dirs & Q6:
		r = reverse_bresenham(array,x-1,y-1,dirs & SW );
		if r != None: 
			array[x,y] = 0;
			return r;
			
	if dirs & Q7:
		r = reverse_bresenham(array,x,y-1,dirs & SOUTH );
		if r != None: 
			array[x,y] = 0;
			return r;				
	if dirs & Q8:
		r = reverse_bresenham(array,x+1,y-1,dirs & SE );
		if r != None: 
			array[x,y] = 0;
			return r;			
	#Couldnt find any remaining pixels
	#returning None here means a line will not terminate unless a vertex is found
	#returning x,y here means that a line is found if it is the end of a line of pixels
	#with no terminating vertex
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
	

def testReverseBresenhamPerformance():
	startPoint = (1,1);
	endPoint = (9,1) ;
	el = 0;
	LOOPS = 1000;
	for i in range(LOOPS):
		a = np.zeros((12,12),dtype=np.uint8 );
		piecewise_bresenham_line(a,startPoint,endPoint );
		s = time.clock();
		q = reverse_bresenham ( a, startPoint[0],startPoint[1] );
		el += ( time.clock() - s );
	
	print "%d iters, %0.5f ms/iter" % ( LOOPS, el );
	
def testReverseBresenham():
	"test cases for finding segments,where the segment is valid"
	startPoint = (1,1);
	endPoints = [ ( 2,2 ),(6,6), (1,9), (9,1) ,(3,8), (8,3), (2,5),(9,7),(2,8),(9,8),(8,9) ];
	#endPoints = [ (9,1) ];
	for p in endPoints:
		print "Test Case %s --> %s " % ( str(startPoint), str(p) ),;
		a = np.zeros((12,12),dtype=np.uint8 );
		piecewise_bresenham_line(a,startPoint,p );
		q = reverse_bresenham ( a, startPoint[0],startPoint[1] );
		if q != p:
			print "Did not find end point" + str(p);
			print q;
			print a;
		else:
			print "Passed."

if __name__=='__main__':
	print "Running Tests....."	
	print "Reverse Bresenham..."; testReverseBresenham();
	print "Reverse Bresenham Performance...";  testReverseBresenhamPerformance();
	print "Piecewise Bresenham...";  testPieceWiseBresenham();
	print "Piecewise Bresenham Performance..."; testPieceWiseBresenhamPerformance();
	
	