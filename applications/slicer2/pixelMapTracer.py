import sys,time
import numpy as np
import math
from PIL import Image,ImageDraw
from collections import deque

"""
	Remember to keep this module simple so it converts to c as well as possible.
	IE, no classes and fancy stuff!
"""

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

#types of pixels we may encounter
DONT_FILL = 0;
BOUNDARY = 1;
MUST_FILL = 2;
MAY_FILL = 3;

STATUS_MASK = 0b00000011;
BOUNDARY_MASK = 0b00000001;
"""
	Inputs: 
	   array: an 8 bit numpy array where each pixel contains information about how to deal with it:
		bits 0 and 1: the fill disposition:
			00= DONTFILL
			01= BOUNDARY
			10= MUST FILL
			11= MAY FILL
		bit 3: fill status:
			 0=not drawn, 
			 1=drawn
		bits 4 -8:
		     contain identifier information used to identify unique edges to follow
		
		x,y, starting indices to search. A starting point in the picture ( must be inside the boundary )
		
		
	Goals:
		Use the information in the pixels to draw all pixes in status 'MUST FILL'
		with as little travel as possible.
		
	Returns:
		Populates returnArray with x,y pairs for the paths.
		typically this will be post processed into lines and arcs
		
"""
def followCurves(array,xStart,yStart,returnArray):

	#search for the nearest pixel which is must fill
	#there may be a better way
	(x,y,path) = findClosestStatus(array,xStart,yStart,MUST_FILL);
	
	#

"""
	follows a single fillpath, returning a list of values.
"""
def followFillPath(sourceArray,(x,y)):
	global cX;
	global cY;
	path = [];
	(xmax,ymax) = sourceArray.shape;
	cX= x;
	cY = y;
	
	def tryAdd(x,y,addToPath=True):
		print "trying (%d,%d)" % ( x,y)
		if sourceArray[x,y] == MUST_FILL:
			global cX;
			global cY;			
			print "add (%d,%d)" % ( x,y)
			sourceArray[x,y] = 0; #set this so we dont find it again
			if addToPath: path.append((x,y));
			cX = x;
			cY = y;
			return True;
		return False;
	
	if not tryAdd(cX,cY,False):
		raise Exception("Start was not a must fill node!");
	
	while True:
		#test each direction. follow the very first path
		#4 connected 
		if tryAdd(cX+1,cY): pass;
		elif tryAdd(cX+1,cY+1): pass;
		elif tryAdd(cX,cY+1): pass;
		elif tryAdd(cX-1,cY+1): pass;
		elif tryAdd(cX-1,cY): pass;
		elif tryAdd(cX-1,cY-1): pass;
		elif tryAdd(cX,cY-1): pass;
		elif tryAdd(cX+1,cY-1): pass;
		else:
			#print "finding..."
			#could not find a connected must fill node, so search for the closest
			#must_fill while avoiding zeros
			#this will take _forever_ as we run out of paths to follow.
			#TODO: have to find a fast end condition
			result = findClosestStatus(sourceArray,(cX,cY),MUST_FILL);
			if result:

				#print "found start of new path at %d,%d" % ( result[0], result[1])
				(cX,cY,newPath) = result; #unpack values
				print "new Path",newPath
				for p in newPath:
					print "add (%d,%d)" % ( p[0],p[1])
					sourceArray[p[0],p[1]] = 0; # so we dont use it again.
				path.extend(newPath); #add to existing path
			else:
				#couldnt find a near point either. return
				return path;

	
#find the closest pixel to the provided one which has the required status
#use breadth first search to avoid recursion depth issues
def findClosestStatus(sourceArray,(x,y),status):
	#print "starting search at (%d,%d)" % ( x, y);
	(xmax,ymax) = sourceArray.shape;
	queue = deque([([],(x,y))]); #stack, empty parent list
	visited = set([(x,y)]); #which ones visited
	
	
	def visit(parentPath, a,b):
		#dont visit if out of bounds
		if a < 1 or b < 1 or a >= xmax or b >= ymax:
			return;
		#dont visit if zero
		if  (a,b) not in visited and (sourceArray[a,b] > 1):
			queue.append ( (parentPath,(a,b)));
			#print "visiting (%d,%d)" % (a,b)
			visited.add((a,b) )
		
	while queue:
		path,(x,y) = queue.popleft();
		parentPath = list(path)
		parentPath.append((x,y));
		
		#print sourceArray[x,y]
		#exit condition when found
		if sourceArray[x,y]  == status:
			return (x,y,parentPath);
	
		#else,continue the search
		#4-connected

		visit(parentPath,x-1,y);
		visit(parentPath,x+1,y);
		visit(parentPath,x,y+1);
		visit(parentPath,x,y-1);
		
		#8-connected
		visit(parentPath,x-1,y-1);
		visit(parentPath,x+1,y-1);
		visit(parentPath,x+1,y+1);
		visit(parentPath,x-1,y+1);
	
	#no path found
	return None;
	
	
"""
	potentially kinda slow implementation when thickness is >1. but maybe it is fast
	enough?
	this implementation implements thickness kind of like a real extruder does-- it
	colors in pixels around the target pixel.
	It is probably not as bad as it seems since numpy slicing does some of the hard work for us
	
"""
def bresham_line(array,value,t,(x0,y0),(x1,y1)):
	steep = abs(y1 - y0) > abs(x1 - x0)
	if steep:
		x0, y0 = y0, x0  
		x1, y1 = y1, x1
		
	if x0 > x1:
		x0, x1 = x1, x0
		y0, y1 = y1, y0

	if y0 < y1: 
		ystep = 1
	else:
		ystep = -1
		 
	deltax = x1 - x0
	deltay = abs(y1 - y0)
	error = -deltax / 2
	y = y0


	for x in range(x0, x1 + 1): # We add 1 to x1 so that the range includes x1
		if steep:
			array[y-t:y+t, x-t:x+t]= value;
		else:
			array[x-t:x+t, y-t:y+t]= value;

		error = error + deltay
		if error > 0:
			y = y + ystep
			error = error - deltax
	
def testFindClosest():
	"""
		test finding closest value in an array
	"""
	test     = np.array([[1,1,1,1,1,1,1,1,1,1], \
						 [1,2,3,0,0,2,0,4,2,1], \
						 [1,0,3,3,3,3,2,8,0,1], \
						 [1,0,3,1,1,1,3,2,0,1], \
						 [1,0,3,1,0,1,3,0,2,1], \
						 [1,2,3,1,1,1,3,2,0,1], \
						 [1,0,2,3,3,3,1,3,2,1], \
						 [1,1,1,1,1,1,1,1,1,1]],dtype=np.uint8 );
	q = time.clock();
	for i in range(1000):
		path = findClosestStatus(test,(1,2),2);
	print "Elapsed: %0.3f/1000 iters" % ( time.clock() - q );
	print path;
	
	print "Follow Fill...."
	q = time.clock();
	path = followFillPath(test,(1,1));
	print "Elapsed: %0.3f/1000 iters" % ( time.clock() - q );		
	print path;
	
def testBresenHamRadially():
	"""
		tests drawing multiple thickness lines
	"""
	CANVAS_SIZE = 1200
	NUM_LINES=100
	a = np.zeros((CANVAS_SIZE,CANVAS_SIZE),dtype=np.uint8 );
	xcenter = int(CANVAS_SIZE/2 );
	ycenter = xcenter;
	margin = CANVAS_SIZE / 10
	line_length = ((CANVAS_SIZE/2) - margin)
	
	angle_step = ( 2 * math.pi ) / NUM_LINES
	q = time.clock();
	
	#draw a whole bunch of lines using bresenham
	for i in range(NUM_LINES):
		theta = (angle_step * i)
		xstart = int(margin * math.cos(theta)) + xcenter
		ystart = int(margin * math.sin(theta)) + ycenter
		xend = int(line_length * math.cos(theta)) + xcenter
		yend = int(line_length * math.sin(theta)) + ycenter
		bresham_line(a, 1, 3,(xstart, ystart), (xend, yend))

	print "%d iters, %0.5f sec " %  (NUM_LINES, (time.clock() - q ));
	#save image out to a file
	tmp = np.array(a,np.uint8 );
	tmp *= 255;
	im = Image.fromarray(tmp,'L');
	im.save("c:\\temp\\testBres.bmp",format="bmp");	


if __name__=='__main__':
	print "Running Tests....."	
	#print "Radial Bresenham Test.. See c:/temp/testRadial.bmp"; testBresenHamRadially();

	print "Test FindCloset..."; testFindClosest();
	