import sys
import time
import numpy as np
import time
from PIL import Image,ImageDraw

def getBoundingBoxForPoints(pnts):
	"pnts is a list of (x,y) tuples"
	def getX(p): return p[0];
	def getY(p): return p[1];
	
	return [ ( min(map(getX,pnts)) ,  min(map(getY,pnts))), \
		( max(map(getX,pnts)) ,  max(map(getY,pnts))) ];

def subtractTuple(t1,t2):
	return (t1[0] - t2[0],t1[1]-t2[1] );
	
"""
	A coordinate transformed pixelmap of a 2-d space
	A numpy int8 array is used so we can store more than just on/off
	Generally, (x,y) tuples are used for coordinates
	
"""
class pixmap:
	def __init__(self,min,max,step):
		self.min = min;
		self.max = max;
		self.step = step;
		
		self.nX = int((max[0] - min[0] ) / step ) + 2;
		self.nY = int((max[1]- min[1] ) / step ) + 2;

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
	
	def drawLine(self,p1,p2,value=1):
		"""
			Draws a line between the two points		
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
		ImageDraw.Draw(i).line([a,b] ,fill=value,width=1.5);
		
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

	def tileOnto(self,arrayToTile):
		"tiles the supplied array over the underlying array"
		(hl, hw ) = arrayToTile.shape;

		(bl, bw ) = self.p.shape;
		numwide = round(bw / hw ) + 1;
		numtall = round(bl / hl ) + 1;

		pattern = np.tile(arrayToTile,(numtall,numwide));
		pattern = pattern[:bl,:bw];

		self.p = np.bitwise_and(self.p,pattern);
		
	def fillTriangle(self,p1,p2,p3,value=1):
		"""
			Draws a closed polygon defined by the set of points.
			
		
		"""
		#assert len(p) > 2,"filling Requires at least 3 points"
		
		"p1, p2, and p3 are 2-tuples having coordinates of the triangle corners"
		#print p1,p2,p3                            

		#get the indexes of the vertices
		i1 = self.index(p1);
		i2 = self.index(p2);
		i3 = self.index(p3);
		#print i1,i2,i3
		
		#get bounding box.
		(min,max) = getBoundingBoxForPoints([i1,i2,i3]);
		#print min,max
		v = self.p[min[0]:max[0]+1,min[1]:max[1]+1];
		#print "bounding box is",v.shape;
	
		#i = Image.new('L',(v.shape[1],v.shape[0]), 0 );
		i = Image.new('L',v.shape, 0 );
		
		#adjust coordinates of the points to coordinates local to the bounding box
		a = subtractTuple(i1,min);
		b = subtractTuple(i2,min);
		c = subtractTuple(i3,min);
		#draw a polygon on the image
		ImageDraw.Draw(i).polygon([a,b,c ],outline=value, fill=value);
		
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
		
		#print m;
		#v[:] = m[:];
		#create a boolean array matching size of v for points having the halfspace
		#equations positive
		
	#TODO: fill a line in space between two segments
	#TODO: get indices of unfilled regions
	def __str__(self):
		return "PixMap: %dx%d, x:(%0.3f-%0.3f) y:(%0.3f-%0.3f)" % \
			( self.nX,self.nY, self.min[0],self.max[0], self.min[1],self.max[1]  );
		
	def saveText(self,fname):
		np.savetxt(fname,self.p,delimiter='',fmt='%d');
		
	def saveImage(self,fname):
		"Save the pixmap to an image, so that we can debug what it looks like"
		#im = Image.frombuffer("1",(self.nX,self.nY),self.p,'raw',"1",0,1);
		a = np.array(self.p,np.uint8 );
		a *= 255;
		im = Image.fromarray(a);
		im.save(fname);
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
	print t.p.shape;
	print t.p;
	print "filled triangle in %0.3f" % (time.clock() - q );
	
	#t.fillTriangle((2.1,2.1),(4.2,4.2),( 4.5,4.5) );
	t.saveImage('c:\\temp\\pixtest.jpg');
