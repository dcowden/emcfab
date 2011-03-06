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
from  OCC import BRepLib
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
		cZ = center[2];
		(XA,YA) = self.lineWidthAdjust();
		baselineY = (cY +  YA ) * positive;
		topY = (cY + ( self.width/2.0 - YA )) * positive ;

		p0 = gp.gp_Pnt(cX -  self.cartesianSpacing()[0], baselineY,cZ);
		p1 = gp.gp_Pnt( cX - self.centerToCorner() + XA, baselineY ,cZ );
		p2 = gp.gp_Pnt( cX - self.halfAflat() - XA, topY , cZ );
		p3 = gp.gp_Pnt( cX + self.halfAflat() + XA , topY, cZ );
		p4 = gp.gp_Pnt(  cX + self.centerToCorner() - XA , baselineY , cZ );
		p5 = gp.gp_Pnt(  cX + self.cartesianSpacing()[0], baselineY , cZ );
	
		#make the edges and the wires
		edges = [];
		edges.append( Wrappers.edgeFromTwoPoints(p0,p1) );
		edges.append( Wrappers.edgeFromTwoPoints(p1,p2) );
		edges.append( Wrappers.edgeFromTwoPoints(p2,p3) );
		edges.append( Wrappers.edgeFromTwoPoints(p3,p4) );
		edges.append( Wrappers.edgeFromTwoPoints(p4,p5) );
		
		wire =  Wrappers.wireFromEdges(edges);
		return wire;
		
	
	def makeHexForBoundingBox(self,bottomLeftCorner,topRightCorner):
		"""
			make a hex array that covers the specified bounding box
			bottomLeftCorner is a tuple (x,y,z) representing the bottom left
			topRightCorner is a tuple (x,y,z) representing the top right.
			
			The algo guarantees that full hexes cover the specified bounding box
		"""
		
		dX = abs(bottomLeftCorner[0] - topRightCorner[0] );
		dY = abs(bottomLeftCorner[1] - topRightCorner[1] );
		
		#how many hexes Wide?
		xCount = ( dX / self.cartesianSpacing()[0] ) + 3;
		yCount = (dY / self.cartesianSpacing()[1]  ) + 2;
		
		#for debug, display the points
		#display.DisplayShape( Wrappers.make_vertex( gp.gp_Pnt(bottomLeftCorner[0],bottomLeftCorner[1],bottomLeftCorner[2])));
		#display.DisplayShape( Wrappers.make_vertex( gp.gp_Pnt(bottomLeftCorner[0],topRightCorner[1],bottomLeftCorner[2])));
		#display.DisplayShape( Wrappers.make_vertex( gp.gp_Pnt(topRightCorner[0],bottomLeftCorner[1],bottomLeftCorner[2])));
		#display.DisplayShape( Wrappers.make_vertex( gp.gp_Pnt(topRightCorner[0],topRightCorner[1],bottomLeftCorner[2])));
		
		#shift the grid into the bounding box a bit
		return self.makeHexArray((  bottomLeftCorner[0] - self.cartesianSpacing()[0], bottomLeftCorner[1] - self.cartesianSpacing()[1]/2.0  ,bottomLeftCorner[2]),xCount,yCount);
	
	def makeHexArray(self,bottomLeftCenter, countX, countY ):
		"""
			makes an array of hexagons
			bottomLeftCenter is the center of the top left hex, as a three-element tuple
			countX is the number of hexes in the x direction
			countY is the number of hexes in the y direction
			returns a list of wires representing a hexagon fill pattern
		"""
		pattern = self.makePeriodic(bottomLeftCenter);
		wireBuilder = BRepBuilderAPI.BRepBuilderAPI_MakeWire(pattern);

		#make horizontal array
		tsf = gp.gp_Trsf();
		pDist = 2.0 * self.cartesianSpacing()[0];
		tsf.SetTranslation(gp.gp_Pnt(0,0,0),gp.gp_Pnt(pDist ,0,0));
		tx = BRepBuilderAPI.BRepBuilderAPI_Transform(tsf);
		currentShape = pattern;
		for i in range(1,int((countX/2)+1)):
			tx.Perform(currentShape,False);
			currentShape = tx.Shape();
			#display.DisplayShape(currentShape);
			wireBuilder.Add(Wrappers.cast(currentShape));

		#create an array by alternately offsetting one cell right and 
		#moving down
		topHalf = wireBuilder.Wire();		
		#topHalf= approximatedWire(topHalf);
		
		wires=[];
		wires.append(topHalf);
		dY = self.cartesianSpacing()[1]/2.0;
		dX = self.cartesianSpacing()[0];
		
		####TODO// performance note.  This method takes about 31ms to compute 1000x1000 hex.
		# pretty good, except that nearly 50% of the time is spent in makeTransform!!!
		# a much better method would be to use the same transform object somehow
		for i in range(1,int(countY*2)):
			if i % 2 == 0:
				t = makeTransform(0,dY*i,0);
			else:
				t = makeTransform(dX,dY*i,0);
			t.Perform(topHalf,False);
			w = Wrappers.cast(t.Shape());
			
			#approximate the wire
			#wires.append ( approximatedWire(w));
			wires.append( w);
		
		#display.DisplayShape(wires);
		return wires;

		
def approximatedWire(wire):
	"returns a bezier approximation of the specified wire as an edge"
	#make a parameterized approximation of the wire
	adaptor = BRepAdaptor.BRepAdaptor_CompCurve (wire);
	curve = BRepAdaptor.BRepAdaptor_HCompCurve(adaptor);
	curveHandle = curve.GetHandle();
	
	#approximate the curve using a tolerance
	approx = Approx.Approx_Curve3d(curveHandle,0.01,GeomAbs.GeomAbs_C2,1000,8);
	if approx.IsDone() and  approx.HasResult():
		# have the result
		anApproximatedCurve=approx.Curve();

		builder =  BRepLib.BRepLib_MakeEdge(anApproximatedCurve);
		e = builder.Edge();
		return Wrappers.wireFromEdges([e]);
		
def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
	p.print_stats(level);			
		
if __name__=='__main__':
	"test this out by doing a basic couple of operations"
	h = Hexagon(2,0 );
	w1 = h.makePeriodic((0,0,0));
	
	h2 = Hexagon(10,.2);
	w2 = h2.makePeriodic((0,0,0));
	
	#display.DisplayShape(w1);
	#display.DisplayShape(w2);
	t = Wrappers.Timer();
	w3 = h2.makeHexForBoundingBox((0,0,0),(100,100,0));
	#runProfiled('w3 = h2.makeHexForBoundingBox((0,0,0),(100,100,0));',0.5);
	print "Total Time: %0.8f " % (t.elapsed()*1000);
	display.DisplayShape(w3 );
	start_display();