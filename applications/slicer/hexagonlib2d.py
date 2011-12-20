"""
	HexLib-- library for dealing with hexagon filling.

"""

import os
import sys
import os.path
import logging
import time
import traceback

import math
from OCC  import gp
from OCC  import BRepAdaptor
from OCC import BRepBuilderAPI
from OCC import Geom
from OCC import BRepBndLib 
from OCC import Bnd;
from OCC import BRep;
from OCC import TopTools
from OCC import TopoDS
from OCC import TopAbs
from OCC import BRepTools
from OCC import Approx
from OCC import GeomAbs
from  OCC import BRepLib,GCE2d
import cProfile
import pstats

"""
My modules
"""
import Wrappers

from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()

log = logging.getLogger('hatchLib');

ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();

#compute this one time, we use it a lot.
SQRT3 = math.sqrt(3);


#make a simple x-y-z transform
def makeTransform(x=0,y=0,z=0):
	"make a simple translation"
	t = gp.gp_Trsf();
	t.SetTranslation( gp.gp_Pnt(0,0,0),gp.gp_Pnt(x,y,z));
	tt=BRepBuilderAPI.BRepBuilderAPI_Transform(t);
	return tt;
	
"""
	Basic computations for hexes
"""
class Hexagon:
	"""
		the hex is orieted with horizontaly aligned flats.
		
		Width is the diameter of an inscribed circle
		
		location is a tuple contining the x and y coordinates of the center, (0,0) if not suppplied
		
		linewidth is the thickness of the border, 0 if not supplied.  upper and lower flats of the hex
		   are shorted by one half this distance to allow two passes on the flats. This is required to make
		   it possible to draw hexes with a continuous pattern.
		   
		thus this hex looks like this:
		
		  /-----\
		 /       \
		 \       /
		  \-----/
		  
	"""
	def __init__(self,width,linewidth=0):
		self.width=width;
		self.linewidth=linewidth;
		
	def diagonal(self):
		"the width across the diagonal"
		return self.width * 2 / SQRT3;
	
	def  distanceBetweenCenters(self):
		"returns the linear distance between centers of two adjacent hexes"
		return self.width;
	
	def centerToCorner(self):
		return self.width / SQRT3;
		
	def cartesianSpacing(self):
		"returns a tuple having the distance between centers in x and y"
		return (  (1.5*self.width/SQRT3), self.width );

	def  _xOffsetByLineWidth(self):
		return (self.width - self.linewidth ) /cc ;
		
	def flat(self):
		"the width of a flat on the hex"
		return self.width / SQRT3;
	
	def lineWidthAdjust(self):
		"returns the x and y adjustment required by the linewidth. returned as a tuple, (x,y)"
		return (  self.linewidth / SQRT3 / 2, (self.linewidth / 2.0) );
		
	def halfAflat(self):
		"the width of 1/2 a flat on the hex"
		return self.flat()/2.0;
	
	def makePeriodic(self,center,positive=1.0):
		"""
			center is the center of the first hex, as an (x,y,z) tuple.
			positive is 1 for the upper portion, -1 for the lower portion
			
			makes the upper part of a periodic hex pattern.
			
			Note that the upper and lower flats are adjusted
			for line width, so that they can be stacked and allow
			double-drawing of the horizontal flats. this offset is controlled by
			the linewidth parameter.
			the points are numbered below
			
			    (2)  (3)
			     /-----\
			____/   +   \_____
		   (0) (1)      (4)   (5)
		"""
		cX = center[0];
		cY = center[1];

		(XA,YA) = self.lineWidthAdjust();
		baselineY = (cY) +  (YA  * positive);
		topY = cY + (( self.width/2.0 - YA ) * positive) ;

		p0 = gp.gp_Pnt2d(cX -  self.cartesianSpacing()[0], baselineY);
		p1 = gp.gp_Pnt2d( cX - self.centerToCorner() + XA, baselineY );
		p2 = gp.gp_Pnt2d( cX - self.halfAflat() - XA, topY );
		p3 = gp.gp_Pnt2d( cX + self.halfAflat() + XA , topY );
		p4 = gp.gp_Pnt2d(  cX + self.centerToCorner() - XA , baselineY  );
		p5 = gp.gp_Pnt2d(  cX + self.cartesianSpacing()[0], baselineY );
	
		#make the edges
		edges = [];
		edges.append( GCE2d.GCE2d_MakeSegment(p0,p1).Value() );
		edges.append( GCE2d.GCE2d_MakeSegment(p1,p2).Value() );
		edges.append( GCE2d.GCE2d_MakeSegment(p2,p3).Value() );
		edges.append( GCE2d.GCE2d_MakeSegment(p3,p4).Value() );
		edges.append( GCE2d.GCE2d_MakeSegment(p4,p5).Value() );
	
		return edges;
		
	
	def makeHexForBoundingBox(self,bottomLeftCorner,topRightCorner):
		"""
			make a hex array that covers the specified bounding box
			bottomLeftCorner is a tuple (x,y,z) representing the bottom left
			topRightCorner is a tuple (x,y,z) representing the top right.
			
			The algo guarantees that full hexes cover the specified bounding box
			Complete re-implementation from the original transformation based version for speed
		"""
		(xMin,yMin) = bottomLeftCorner
		(xMax,yMax) = topRightCorner
		
		dX = abs(bottomLeftCorner[0] - topRightCorner[0] );
		dY = abs(bottomLeftCorner[1] - topRightCorner[1] );
		
		#spacing of the pattern horizontally and vertically
		yIncrement = self.cartesianSpacing()[1];
		xIncrement = self.cartesianSpacing()[0]*2.0;
		
		rows = []
		for y in Wrappers.frange6(yMin,yMax,yIncrement):
			curves = []
			#make the bottom row
			for x in Wrappers.frange6(xMin,xMax,xIncrement):
				edges = self.makePeriodic( (x,y), -1 );
				curves.extend(edges);
			rows.append(curves);
			curves = [];
			
			#make the upper row
			for x in Wrappers.frange6(xMin,xMax,xIncrement):
				edges = self.makePeriodic( (x,y), 1);
				curves.extend(edges);
			rows.append(curves);  #list of list of lines

		return rows;

def displayCurve(curve):
	"""
		Displays a curve. kind of a pain, since curves must be converted to edges first
	"""
	edge = BRepBuilderAPI.BRepBuilderAPI_MakeEdge2d(curve).Edge()
	display.DisplayShape(edge,update=False);
	
def displayWire(wire):
	for c in wire:
		displayCurve(c)
		
def displayHexArray(array):
	"a hex array in this implementation is a list of list of curves"
	for a in array:
		displayWire(a);

def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
	p.print_stats(level);			
		
if __name__=='__main__':
	"test this out by doing a basic couple of operations"

	h2 = Hexagon(10,.2);
	w2 = h2.makePeriodic((0,5),1);
	#displayWire(w2)
	w2 = h2.makePeriodic((0,5),-1);
	#displayWire(w2);
	#display.DisplayShape( Wrappers.make_vertex(gp.gp_Pnt(0,0,0)))
	#TestDisplay.display.run();
	#display.DisplayShape(w1);
	#display.DisplayShape(w2);
	q = time.clock()
	w3 = h2.makeHexForBoundingBox((0,0),(100,100));
	#runProfiled('w3 = h2.makeHexForBoundingBox((0,0),(100,100));',0.8);
	print "Made 1 arrays in %0.8f " % (time.clock() - q);
	displayHexArray(w3);
	display.FitAll();
	start_display();