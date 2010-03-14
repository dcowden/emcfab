"""
	Routines for filling a cross section with toolpaths
"""
import string;





"""
	a grid that stores pixels that are used to track if a 
	shape is finished
	
	A primary function is to efficiently determine if a values
	is contained in the grid, even if the value requested is 
	not exactly in the grid.
	
	IE, each arbitrary point should be mapped to exactly one
	grid section if it is close ao any grid section
	
	idea: rounding ( base 10 ) is a way to very quickly convert
	a datapoint to a lesser precision. A grid is the same thing,
	except the precision is not a power of 10. So, can we multiply
	all the values by width/10, and then round?
	
"""
class BooleanGrid:
	def __init__(self,precision):
		self.precision = precision;
		self.keyformat = "%d-%d";
		
		#the grid is composed of parametric space scaled so that rounding efficiently finds
		#matching points
		
		#contains boolean values for whether a paricular coordinate is covered or not.
		#coordinates are stored in pspace, which is a space multiplied by a scaling factor
		#such that finding a point closest is simply doing the rounding function
		self.pixels = {}; #x then y
	
	def pointsFromKey(self,key):
		s = string.split(key,"-");
		return [ float(s[0])*self.precision,float(s[1])*self.precision ];
	def _makeKey(self,x,y):
		#convert the values to pspace then look them up
		pX = round(x/self.precision);
		pY = round(y/self.precision);
		k = self.keyformat % ( pX, pY );
		return k;
	
	def pointCount(self):
		return len(self.pixels);
		
	def clear(self):
		for k in self.pixels.keys():
			self.pixels[k] = false;

	#def getItemsMatching(self,value):
	#	d = 
	#	for k,v in self.pixels.iteritems():

	#a boolean if the point is in the grid
	#if create is true, the point is created if it does not exist.
	#if create is false, None is returned if it does not exist.
	def getPoint(self,x,y):
		return self.pixels.get(self._makeKey(x,y));

	def setPoint(self,x,y,value):
		self.pixels[self._makeKey(x,y)] = value;
		
if __name__=='__main__':
	
	grid = BooleanGrid(0.005 );
	print "Running Test Cases with a 0.005 spaced grid"
	#set up a few points that are in the grid
	# looks like this ( each cursor is a grid space ):
	"""
	
	      X
		 X X
		X  X 
	
	"""
	print "Basic Assertions..."
	grid.setPoint(0.6,0.6,False);  #grid point 1
	grid.setPoint(0.605,0.605,False); #grid point 2
	grid.setPoint(0.610,0.600,False); #grid point 4
	grid.setPoint(0.610,0.605,False); #grid point 5
	
	#assert ( 0.61234 == grid.pointsFromKey(grid._makeKey(0.61234,0.6))[0]),"Keys should be equal"
	
	print "Reciprocal Lookups..."
	a =  grid._makeKey(0.61234,0.6);
	b =  grid.pointsFromKey(a);
	assert b[0] == 0.610, "x value should match 0.610";
	assert b[1] == 0.600, "y value should match 0.600";
	
	assert (grid.getPoint(0.6,0.6) != None),"0.6,0.6 Should be in grid point 1";
	assert ( grid.getPoint(0.600202,0.60249) != None),"0.600202,0.60249 Should be in grid point 1";
	assert (grid.getPoint(0.594,0.594) == None ),"Should not match anything" ;
	assert (grid.getPoint(0.594,0.599) == None ),"Should not match anything" ;
	
	#now toggle a point and see if it works
	print "Mutation...."
	grid.setPoint(0.6018,0.602,True);	
	assert  (grid.getPoint(0.598,0.599) == True ),"Should match gridpoint 1";
	print "Test Cases Passed.";
