"""
	Provides Functions that traverse
	wires and edges in an efficient way.
	
	This class manages the state of moving along
	a set of wires and edges.
	
	This module forms the basis of a package to efficiently navigate
	edges, shapes, and such for toolpaths
"""

import os
import sys,logging
from OCC import TopoDS, TopExp, TopAbs,BRep,gp,GeomAbs,GeomAPI,BRepBuilderAPI

import Wrappers
import Utils

topoDS = TopoDS.TopoDS();

###Logging Configuration
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
                    stream=sys.stdout)

log = logging.getLogger('slicer');
log.setLevel(logging.WARNING);

"""
	A Linear move to a particular point
	The move may have the pen down or not
	
	FromPoint can be null if there is no defined current point
"""
class LinearMove:
	def __init__(self, fromPoint, toPoint,draw=True):
		self.toPoint = toPoint;
		self.draw = draw;
		if fromPoint:
			self.dir = gp.gp_Vec(fromPoint,toPoint);
	
	def __str__(self):
		if self.draw:
			s = "DrawTo:";
		else:
			s = "MoveTo:";
		return s + str(Wrappers.Point(self.toPoint));
		
"""
    A move in an arc to another point.
	Arc moves have an endpoint, a center poitn, and are clockwise or counterclockwise
"""
class ArcMove():
	def __init__(self,endPoint,centerPoint,ccw):
		self.centerPoint = centerPoint;
		self.endPoint = endPoint;
		self.ccw = ccw;

	def __str__(self):
		return "ArcMove: ccw=" + str(self.ccw) + ", centerPoint=" + str(Wrappers.Point(self.centerPoint)) + ", toPoint = " + str(Wrappers.Point(self.endPoint));		
		
"""
  Follows edges and wires, generating 
  a series of moves to allow converting them
  into another language or format.
  
  This class provides a few functions that 
  are central:
  
  * maintain state to avoid duplicate moves
  * convert curves to the appropriate stream of points
  * provide a generator that allows easily navigating without using memory
  * follow lists of edges, wires, and shapes in the order provided
  * abstract moves into structures for other libs to use ( svg and gcode )
"""	
class ShapeFollower():
	def __init__(self,useArcs):
		"useArcs determines if we should generate ArcMoves or not"
		self.currentPoint = None;
		self.pendingLinearMove = None;
		self.tolerance = 0.0001;
		self.approximatedCurveDeflection = 0.01;
		self.useArcs = useArcs;
		
	
	def isClose(self,point):
		"is the specified point close to the current point"
		if self.currentPoint:
			return self.currentPoint.Distance(point) < self.tolerance;
		return False;

	def _fetchPendingMove(self):
		t = self.pendingLinearMove;
		self.pendingLinearMove = None;
		return t;
		
	def follow(self,shapeList, finish=True):
		"a generator that returns a list of moves that navigates the list of shapes"
		"if finish is true, any pending move will be finished. If false,"
		"pending linear moves are left in case next moves duplicate them"
		log.debug("Following Shape");
		for s in shapeList:
			for m in self.followShape(s):
				yield m;
		
		#flush any last remaining move
		if finish:
			t= self._fetchPendingMove();
			if t:
				yield t;
					
	def followShape(self,shape,finish=False):
		"follows  this shape"
		if shape.ShapeType() == TopAbs.TopAbs_WIRE:
			for m in self.followWire(topoDS.Wire(shape)):
				yield m;
		elif shape.ShapeType() == TopAbs.TopAbs_EDGE:
			for m in self.followEdge(topoDS.Edge(shape)):
				yield m;
		elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
			bb = TopExp.TopExp_Explorer();
			bb.Init(shape,TopAbs.TopAbs_WIRE);
			while bb.More():
				w = topoDS.Wire(bb.Current());
				for m in self.followWire(wire):
					yield m;
				bb.Next();			
			bb.ReInit();		
		else:
			"unknown shape"
			pass;
			
		#flush any last remaining move
		if finish:
			t= self._fetchPendingMove();
			if t:
				yield t;
			
	def followWire(self,wire,finish=False):
		"""
			Follow the edges in a wire in order
		"""	
		ww = Wrappers.Wire(wire);
		for e in ww.edges():
			for m in self.followEdge(e,False):
				yield m;

		#flush any last remaining move
		if finish:
			t= self._fetchPendingMove();
			if t:
				yield t;
	
	def followEdge(self,e,finish=False):
		"""
		return a sequence of points for an edge as a generator
		NOTE: this algorithm leaves a potential linear move not returned,
		so you have to check pendingLinearMove at the end and return  it if not null
		"""
		
		ew = Wrappers.Edge(e);
		log.debug("Follow Edge: " + str(ew));
		
		#if no current point, move to the first edge.
		if not self.currentPoint:
			log.debug("No Current Point, Move to start.");
			"no current point defined yet."
			self.currentPoint = ew.firstPoint;
			yield LinearMove(None,ew.firstPoint,False);
		else:
			"there is a current point. But if the start of the edge is not close, we have to move there"
			if self.isClose(ew.firstPoint):
				log.debug("No moved Required to get to edge start.");
			else:
				log.debug("Must Move to first point, running any previous move.");
				t= self._fetchPendingMove();
				p = None;
				if t:
					p = t.toPoint;
					yield t;
				
				log.debug("Moving to First Point of Edge");
				self.currentPoint = ew.firstPoint;
				yield LinearMove(p,ew.firstPoint,False);
				
		if ew.isLine():
			"the last move was a line, and this one is too."
			"if this move is parallel to the last one, then we'll dump that one"
			"and replace it with this one. otherwise, we'll execute it first"
			thisMoveDir = gp.gp_Vec(ew.firstPoint,ew.lastPoint);
			if self.pendingLinearMove:				
				if  not self.pendingLinearMove.dir.IsParallel(thisMoveDir,self.tolerance):
					"this move is parallel to the last one, so replace with this one"
					log.debug("Move is redundant, replacing with this one.");
					yield self.pendingLinearMove;

			log.debug("Saving Move as Pending.");
			self.pendingLinearMove = LinearMove(ew.firstPoint,ew.lastPoint,True);
			self.currentPoint = ew.lastPoint;
		else:
			t= self._fetchPendingMove();
			if t:
				yield t;

			if ew.isCircle() and self.useArcs:
				log.warn("Curve is a Circle");
				circle = ew.curve.Circle();
				center = circle.Location();

				axisDir = circle.Axis().Direction();				
				#assume we are essentially in 2d space, so we're looking only at wehter the
				#axis of the circle is +z or -z
				zDir = gp.gp().DZ();
				ccw = zDir.IsEqual(axisDir,self.tolerance);
				if ew.reversed:
					zDir = zDir.Reversed();
				self.currentPoint = ew.lastPoint;
				yield ArcMove(ew.lastPoint,center,ccw);				
			else:
				log.warn("Curve is not a line or a circle");
				"a curve, or a circle we'll approximate"
				lastPoint = None;
				for p in ew.discretePoints(self.approximatedCurveDeflection):
					yield LinearMove(lastPoint,p,True);
					lastPoint = p;
				self.currentPoint = lastPoint;
		#flush any last remaining move
		if finish:
			t= self._fetchPendingMove();
			if t:
				yield t;	
				

def testMoves(shapeList):
	"returns the number of moves in the list provided"
	moves = [];
	f = ShapeFollower(True);
	for move in f.follow(shapeList):
		print move;
		moves.append(move);		
	return len(moves);

def makeTestWire():
    circle2 = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(40,40,2),gp.gp_Dir(0,0,1)),10)
    Edge4 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle2,gp.gp_Pnt(40,50,2),gp.gp_Pnt(50,40,2)).Edge()
    ExistingWire2 = BRepBuilderAPI.BRepBuilderAPI_MakeWire(Edge4).Wire()
    P1 = gp.gp_Pnt(50,40,2)
    P2 = gp.gp_Pnt(80,40,2) #5,204,0
    Edge5 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(P1,P2).Edge()
    MW = BRepBuilderAPI.BRepBuilderAPI_MakeWire()
    MW.Add(Edge5)
    MW.Add(ExistingWire2)

    if MW.IsDone():
	WhiteWire = MW.Wire()
	return WhiteWire;	
if __name__=='__main__':
	"PathExport: A Module for Navigating Wires, Edges, and Shapes"
	print "Running Test Cases..."
	
	print "Connected Edges",
	edge1 = Utils.edgeFromTwoPoints(gp.gp_Pnt(0,0,0),gp.gp_Pnt(1,1,0));
	edge2 = Utils.edgeFromTwoPoints(gp.gp_Pnt(1,1,0),gp.gp_Pnt(2,2,0));
	edge3 = Utils.edgeFromTwoPoints(gp.gp_Pnt(2,2,0),gp.gp_Pnt(3,2.2,0));
	#the correct answer is:
	# MoveTo 0,0
	# DrawTo 2,2
	# DrawTo 3,2.2
	assert testMoves([edge1,edge2,edge3]) == 3,"There should be Thee Moves"
	print "[OK]"
	
	print "Disconnected Edges",
	#the correct answer is:
	#  MoveTo 0,0
	#  Lineto 1,1
	#  MoveTo 2,2
	#  LineTo 3,2.2
	assert testMoves([edge1,edge3]) == 4,"Should be four moves"
	print "[OK]"
	
	print "Arcs And Lines",
	wire = makeTestWire();
	#the correct answer is:
	#  Moveto 40,50
	#  ArcTo  50,40 with center 40,40, ccw direction
	#  LineTo 80,40
	assert testMoves([wire]) == 3,"Should be 20 moves?"
	print "[OK]"
	