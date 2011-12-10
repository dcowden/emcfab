import sys
import time
import numpy as np
import time
import bresenham as bres
import networkx as nx

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
		
		self.nX = int((max[0] - min[0] ) / step + 2) ;
		self.nY = int((max[1]- min[1] ) / step  + 2) ; #+2 just corrects for rounding

		#self.p = np.zeros( (self.nX,self.nY),dtype=np.uint8 );
		self.p = np.zeros ( ( self.nX, self.nY),dtype=np.int );
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
	
	
	def drawLine2(self,p1,p2):
		"""
			draws a line, but while doing so, marks any pixels that have already been drawn as vertices
			TODO: lots of code copied here from drawLine, but i'm just prototyping for now
			
		"""
		#TODO: a lot of this code is similar to fillTriangle.
		#TODO: should be refactored.
		i1 = self.index(p1);
		i2 = self.index(p2);
		#draw a line on the canvas, while promoting any pixels that have already been drawn 
		#to vertices.  if the underlying pixel-based tile has vertice built-in, then this 
		#will retain those, while also promoting crossed edges to vertices
		#newVertices = [i1];
		#newVertices.extend(bres.drawLineReturnVertices(self.p,i1,i2,value,VERTEX_VALUE));
		return bres.piecewise_bresenham_line(self.p,i1,i2,20);
		#newVertices.append(i2);
		#return newVertices;
		
	def drawLine(self,p1,p2,value=1,width=1):
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

	
	def followFill(self ,startPoint):
		"""
			attempt to follow all the contours in the graph
		"""
		ar = self.p;
		
		#how many pixels to fill?
		(ix,iy) = np.where(self.p == 1);
		pointsToFill = zip(ix,iy);
		numPoints = len(pointsToFill);
		pointList = [];
		N = [(-1,1),(0,1),(1,1)];
		W = [(-1,-1),(-1,0),(-1,1)];
		S = [(1,-1),(0,-1),(-1,-1)];
		E = [(1,1),(1,0),(1,-1)];
		dirs = [N,W,S,E];
		(x,y) = startPoint;		
		currentDir = N;
		currentDirNum = 0;
		
		#functions to help tracking
		def rotateCCW():
			if currentDirNum == 0:
				currentDirNum == 3;
			else:
				currentDirNum -= 1;
			currentDir = dirs[currentDirNum];

		def P(pnt,i):
			return ((pnt[0] + currentDir[i][0]),(pnt[1] + currentDir[i][1] ));			
		def P1(pnt):
			return P(pnt,0);
		def P2(pnt):
			return P(pnt,1);
		def P3(pnt):
			return P(pnt,2);
		
		
		
	def makeGraph(self,vertexValue=9):
		"""
			makes a graph of the map, based on the vertices.
			The vertex value is used to identify vertices, and edge identifiers
			are assumed to connect vertices together.
			
			Edges are also created between adjacent vertices if desired.
		"""
		ar = self.p;
		g = nx.Graph();
		
		#get all of the vertices in the map. these are just the border vertices
		(ix,iy) = np.where(self.p == vertexValue);
		vertices = zip(ix,iy);

		print "found %d vertices " % len(vertices);		
		#for each vertex, add to the graph, and then follow the paths around it
		#note that all the vertices in this list are just the ones that intersect borders.
		#this approach is very very crude: it will attempt to re-trace many lines too many times,
		#but this gives an idea how fast this approach will be, to some approximation
		for v in vertices:
			g.add_node(v);
			
			#look for paths around this node, as well as immediately neighboring vertices
			for loc in neighbors(v):
				val = ar[loc];
				if val == 8 or val == 6 or val == 9:
					#connection to an internal vertex
					g.add_edge(v,loc);
				elif ar[loc] > 0:
					#a path to follow
					endloc = bres.follow_edge(ar,v[0],v[1],val);
					if endloc != None:
						g.add_edge(v,endloc);
			
			#done with this vertex
			#ar[v] = 0;
		return g;
		
	def tileOnto(self,arrayToTile):
		"tiles the supplied array over the underlying array"
		(hl, hw ) = arrayToTile.shape;

		(bl, bw ) = self.p.shape;
		numwide = round(bw / hw ) + 1;
		numtall = round(bl / hl ) + 1;

		pattern = np.tile(arrayToTile,(numtall,numwide));
		pattern = pattern[:bl,:bw];

		self.p = np.bitwise_and(self.p,pattern);
		
	def fill(self,seedPoint,fillValue=1 ):
		"fills an area starting with seed"
		i = self.index(seedPoint);
		v = self.p[:];
		x = Image.fromarray(v.T);
		ImageDraw.Draw(x);
		ImageDraw.floodfill(x,(i[0],i[1]),fillValue);

		v[:] = np.array(x).T;
		
	def drawPolygon(self,points,outlineFillValue=1,innerFillValue=1):
		"""
			Draws a closed polygon defined by the set of points.
			if the bg color is specified, then it is used for the background color.
			
			It would be really really nice if the PIL version of polygon filling
			worked, but it doesnt, floodfill seems to work, though, we we use that
			
		"""
		#assert len(p) > 2,"filling Requires at least 3 points"
		
		"p1, p2, and p3 are 2-tuples having coordinates of the triangle corners"
		indexes = map(self.index,points);

		#get bounding box.
		(min,max) = getBoundingBoxForPoints(indexes);
		#print min,max
		v = self.p[min[0]:max[0],min[1]:max[1]];
		#print "bounding box is",v.shape;
	
		#i = Image.new('L',(v.shape[1],v.shape[0]), 0 );
		#i = Image.new('L',v.shape, bg );
		i = Image.fromarray(v.T);
		#print v.shape;
		#print i;
		#adjust coordinates of the points to coordinates local to the bounding box
		def sub( i ):
			return (i[0] - min[0],i[1]-min[1] );
		
		adjusted = map(sub,indexes )
		#draw a polygon on the image
		#dont use fill values on this object, it results in buggy patterns.
		ImageDraw.Draw(i).polygon(adjusted, outline=outlineFillValue,fill=innerFillValue);
		#ImageDraw.Draw(i).polygon(adjusted, outline=outlineFillValue);	
		#copy values into windowed box
		#must or the values to avoid overwriting previously set pixels
		#m = np.array(i);
		#print "image array shape is",m.shape
		
		m = np.array(i).T;
		#m /= 255;
		#print m.shape;
		#m /= 255;
		#v[:] = np.maximum(v,m);
		v[:] = m;
		
		
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
		im = Image.fromarray(a,'L');
		im.save(fname,format="bmp");
		del im;
		
	
	
if __name__=='__main__':
	"test cases"
	q = time.clock();
	t = pixmap((0,0),(3.5,2.5),0.001 );
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
