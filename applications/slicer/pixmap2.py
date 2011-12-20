import sys
import time
import numpy as np
import time
import breshamtest as bres

"""

	Designed to allow conversion to pure-c routines for super speed in the future.
	Note: some older routines are in pixmaptest.py-- i removed them for now

	A coordinate transformed pixelmap of a 2-d space
	A numpy int8 array is used so we can store more than just on/off.
	Typically this may be a counter used to cross-reference against objects in a dictionary.
	I resisted using complex object types for the content because use of integers allows
	direct visualization of the pixelmap as an image.
	
	Generally, (x,y) tuples are used for coordinates	
	
"""

from PIL import Image,ImageDraw

def neighbors((x,y)):
	"returns a list of the pixels connected to this one"
	return [ (x+1,y),(x+1,y+1),(x,y+1),(x-1,y+1),(x-1,y),(x-1,y-1),(x,y-1),(x+1,y-1) ];
		
def getBoundingBoxForPoints(pnts):
	"pnts is a list of (x,y) tuples"
	def getX(p): return p[0];
	def getY(p): return p[1];
	
	return [ ( min(map(getX,pnts)) ,  min(map(getY,pnts))), \
		( max(map(getX,pnts)) ,  max(map(getY,pnts))) ];

def subtractTuple(t1,t2):
	return (t1[0] - t2[0],t1[1]-t2[1] );
	

class pixmap:
	def __init__(self,min,max,step):
		self.min = min;
		self.max = max;
		self.step = step;
		
		self.nX = int((max[0] - min[0] ) / step + 2) ;
		self.nY = int((max[1]- min[1] ) / step  + 2) ; #+2 just corrects for rounding

		self.p = np.zeros( (self.nX,self.nY),dtype=np.uint8 );
		#self.p.shape = self.nY,self.nX;
		
	def index(self,p ):
		return (self.indexOf(p[0],self.min[0]), self.indexOf(p[1],self.min[1]));

	def coordOf(self,i,axisMin):
		"get the coordinate of a value given the index"
		return ( self.step * i ) + axisMin;
		
	def coord(self,p):
		"get the coordinates of the given tuple"
		return (self.coordOf(p[0],self.min[0]),self.coordOf(p[1],self.min[1]));
		
	def indexOf(self,n,axisMin):
		"get the index of a value in an axis"
		return np.floor_divide((n - axisMin),self.step);
		
	def get(self,p):
		return self.p[self.index(p)];
	
	def set(self,p,value):
		self.p[self.index(p)] = value;		
				
	def getBox(self,min,max):
		"get a view of the underlying data for a given window"
		p1 = self.index(min);
		p2 = self.index(max);
		v = self.p[p1[0]:p2[0], p1[1]:p2[1]];
		return  v
	
	def drawLine(self,p1,p2,value=1,width=1):
		"""
			Draws a line between the two points	, with optional thickness. 'width'	
		"""
		
		#TODO: a lot of this code is similar to fillTriangle.
		#TODO: should be refactored.
		i1 = self.index(p1);
		i2 = self.index(p2);
		(min,max) = getBoundingBoxForPoints([i1,i2] );
		v = self.p[min[0]:max[0]+1,min[1]:max[1]+1];

		i = Image.new('L',v.shape, 0 );
		
		#adjust coordinates of the points to coordinates local to the bounding box
		a = subtractTuple(i1,min);
		b = subtractTuple(i2,min);

		#draw a polygon on the image
		ImageDraw.Draw(i).line([a,b] ,fill=value,width=width);
		
		#copy values into windowed box
		#must or the values to avoid overwriting previously set pixels
		#m = np.array(i);
		#print "image array shape is",m.shape
		#m /= 255;
		m = np.array(i).T;
		#print m;
		
		#print m.shape;
		#m /= 255;
		v[:] = np.maximum(v,m);

	
		
	#TODO: fill a line in space between two segments
	#TODO: get indices of unfilled regions
	def __str__(self):
		return "PixMap: %dx%d, x:(%0.3f-%0.3f) y:(%0.3f-%0.3f)" % \
			( self.nX,self.nY, self.min[0],self.max[0], self.min[1],self.max[1]  );
		
	def saveText(self,fname):
		np.savetxt(fname,self.p,delimiter='',fmt='%d');
		
	def saveImage(self,fname):
		"Save the pixmap to an image, so that we can debug what it looks like"
		a = np.array(self.p,np.uint8 );
		a *= 255;
		im = Image.fromarray(a,'L');
		im.save(fname,format="bmp");
		del im;
		
	
	
if __name__=='__main__':
	"test cases"
	q = time.clock();
	t = pixmap((0,0),(3.5,2.5),0.01 );
	t.set((1.301,0.18),1);
	#print "Shape is",t.p.shape
	print "made map in %0.3f" % (time.clock() - q );
	q = time.clock();
	#xx = t.getBox((0.1,0.4),(4.2,2.2))[:] = 1;
	#yy = t.getBox((4.0,5.5),(4.0,5.5))[:] = 1;

	t.fillTriangle(( 0.3,0.44),(1.4,.3),(.6,1.8) );
	t.drawPolygon([(1.0,1.0),(1.1,1.0),(2.0,1.5),(2.5,2.0)]);
	print t.p.shape;
	print t.p;
	print "filled triangle in %0.3f" % (time.clock() - q );
	
	#t.fillTriangle((2.1,2.1),(4.2,4.2),( 4.5,4.5) );
	t.saveImage('c:\\temp\\pixtest.jpg');
	
	t2 = pixmap((0,0),(5.0,5.0),0.01);
	t2.drawPolygon([(1.0,1.0),(2.0,2.0),(1.0,2.0)],1,0 );
	
	t2.saveImage('c:\\temp\\pixtest-1.bmp');

	t2 = pixmap((0,0),(5.0,5.0),0.01);
	t2.drawPolygon([(1.0,1.0),(2.0,2.0),(1.0,2.0)],0,1 );
	t2.saveImage('c:\\temp\\pixtest-2.bmp');	

	t2 = pixmap((0,0),(5.0,5.0),0.01);
	t2.drawPolygon([(1.0,1.0),(2.0,2.0),(1.0,2.0)],-1,1 );
	t2.saveImage('c:\\temp\\pixtest-4.bmp');		

	t2 = pixmap((0,0),(5.0,5.0),0.01);
	t2.drawPolygon([(1.0,1.0),(2.0,2.0),(1.0,2.0)],0,1);
	t2.saveImage('c:\\temp\\pixtest-5.bmp');	
	
	t2 = pixmap((0,0),(5.0,5.0),0.01);
	t2.drawLine((1.0,1.0),(2.0,2.0),1 );
	t2.drawLine((2.0,2.0),(1.0,2.0),1 );
	t2.drawLine((1.0,2.0),(1.0,1.0),1 );
	t2.saveImage('c:\\temp\\pixtest-3.bmp');
	
	#jeeze. finally eliminate all my code and see if it does this crap
	i = Image.new('RGB',(50,50), 0 );
	ImageDraw.Draw(i).polygon([(10,10),(20,20),(10,20) ],outline='red',fill='green');
	ImageDraw.Draw(i).polygon([(10,10),(20,20),(20,10) ],outline='red',fill='green');
	#ImageDraw.floodfill(i,(13,17),255);
	i.save("c:\\temp\\freak1.bmp",format="bmp");
	
	#jeeze. finally eliminate all my code and see if it does this crap
	i = Image.new('L',(50,50), 0 );
	ImageDraw.Draw(i).polygon([(10,10),(20,20),(10,20) ],outline=0, fill=255);
	i.save("c:\\temp\\freak2.bmp",format="bmp");	
