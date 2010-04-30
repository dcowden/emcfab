"""
	Provides Functions that traverse
	wires and edges in an efficient way.
	
	This class manages the state of moving along
	a set of wires and edges.
	
	This module forms the basis of a package to efficiently navigate
	edges, shapes, and such for toolpaths
"""

import os,math
import sys,logging
from OCC import TopoDS, TopExp, TopAbs,BRep,gp,Geom,GeomAbs,GeomAPI,BRepBuilderAPI

import Wrappers
import TestDisplay
topoDS = TopoDS.TopoDS();



log = logging.getLogger('PathExport');


"""
	A Move of some kind
"""
class Move:
	def __init__(self,fromPoint,toPoint):
	
		if fromPoint == None:
			fromPoint = gp.gp_Pnt(0,0,0);
			
		self.fromPoint = fromPoint;
		self.toPoint = toPoint;
		self.dir = gp.gp_Vec(fromPoint,toPoint);
	


	def vector(self):
		"returns a vector for this move. if fromPoint is empty, assumes the origin"
		return [ self.toPoint.X(), self.toPoint.Y(), self.toPoint.Z() ];
	
	def distance(self):
		"returns a triple that represents the distance the move is on each axis"
		return [  self.toPoint.X() - self.fromPoint.X(), 
				  self.toPoint.Y() - self.fromPoint.Y(),
				  self.toPoint.Z() - self.fromPoint.Z() ];
"""
	A Linear move to a particular point
	The move may have the pen down or not
	
	FromPoint can be null if there is no defined current point
"""
class LinearMove(Move):
	def __init__(self, fromPoint, toPoint,draw=True):
		Move.__init__(self,fromPoint,toPoint);
		self.draw = draw;

	def length(self):
		return self.toPoint.Distance(self.fromPoint );
		
	def __str__(self):
		if self.draw:
			s = "DrawTo: ";
		else:
			s = "MoveTo: ";
		return s + str(Wrappers.Point(self.toPoint));
		
"""
    A move in an arc to another point.
	Arc moves have an endpoint, a center point, and are clockwise or counterclockwise
"""
class ArcMove(Move):
	def __init__(self,fromPoint,toPoint,centerPoint,ccw,includedAngle):
		Move.__init__(self,fromPoint,toPoint);
		self.centerPoint = centerPoint;
		self.ccw = ccw;
		self.includedAngle = includedAngle;
		
	def length(self):
		return self.includedAngle * self.getRadius();
		
	def getRadius(self):
		"gets the radius of the circle"
		dX = abs( self.fromPoint.X() - self.centerPoint.X() );
		dY = abs ( self.fromPoint.Y() - self.centerPoint.Y() );
		return math.sqrt( math.pow( dX, 2) + math.pow(dY,2) );
		
	def __str__(self):
		if self.ccw:
			s = "CCW ArcTo: ";
		else:
			s = "CW ArcTo: ";
		return s + str(Wrappers.Point(self.toPoint)) + ", center=" + str(Wrappers.Point(self.centerPoint)) ;		

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
class ShapeDraw():
	def __init__(self,useArcs,curveDeflection ):
		"useArcs determines if we should generate ArcMoves or not"
		self.currentPoint = None;
		self.pendingLinearMove = None;
		self.tolerance = 0.001;
		self.approximatedCurveDeflection = curveDeflection;
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
		for e in ww.edges2():
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
					log.info("Move is redundant, replacing with this one.");
					yield self.pendingLinearMove;

			log.debug("Saving Move as Pending.");
			self.pendingLinearMove = LinearMove(ew.firstPoint,ew.lastPoint,True);
			self.currentPoint = ew.lastPoint;
		else:
			t= self._fetchPendingMove();
			if t:
				log.debug("Flushing a pending linear move...");
				yield t;

			if ew.isCircle() and self.useArcs:
				log.debug("Curve is a Circle");
				circle = ew.curve.Circle();
				center = circle.Location();

				axisDir = circle.Axis().Direction();				
				#assume we are essentially in 2d space, so we're looking only at wehter the
				#axis of the circle is +z or -z
				zDir = gp.gp().DZ();
				
				if ew.reversed:
					zDir = zDir.Reversed();
				ccw = zDir.IsEqual(axisDir,self.tolerance);	
				self.currentPoint = ew.lastPoint;
				
				includedAngle = abs( ew.firstParameter - ew.lastParameter );
				a = ArcMove(ew.firstPoint, ew.lastPoint, center, ccw ,includedAngle);
				#circles are parameterized between zero and 2*pi.
				yield a;
				#yield  ArcMove(ew.firstPoint, ew.lastPoint, center, ccw );				
			else:
				log.debug("Curve is not a line or a circle");
				"a curve, or a circle we'll approximate"
				#the generated points might include the beginning of the curve,
				#so test for that, but only for the first point
				firstP = True;
				lastPoint = None;
				for p in ew.discretePoints(self.approximatedCurveDeflection):
					#check the first generated point for a duplicate
					if firstP and self.isClose(p):
						firstP = False;
						continue;
					
					yield LinearMove(lastPoint,p,True);
					lastPoint = p;

				self.currentPoint = lastPoint;
				
		#flush any last remaining move
		if finish:
			t= self._fetchPendingMove();
			if t:
				yield t;	
				

def testMoves(shapeList,useArcs=True):
	"returns the number of moves in the list provided"
	moves = [];
	f = ShapeDraw(useArcs,0.01);
	for move in f.follow(shapeList):
		print "\t",move;
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

	###Logging Configuration
	logging.basicConfig(level=logging.WARN,
						format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
						stream=sys.stdout)
	"PathExport: A Module for Navigating Wires, Edges, and Shapes"
	print "Running Test Cases..."
	
	print "Connected Edges"
	edge1 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(0,0,0),gp.gp_Pnt(1,1,0));
	edge2 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(1,1,0),gp.gp_Pnt(2,2,0));
	edge3 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(2,2,0),gp.gp_Pnt(3,2.2,0));
	#the correct answer is:
	# MoveTo 0,0
	# DrawTo 2,2
	# DrawTo 3,2.2
	assert testMoves([edge1,edge2,edge3]) == 3,"There should be Thee Moves"
	print "[OK]"
	
	print "Disconnected Edges"
	#the correct answer is:
	#  MoveTo 0,0
	#  Lineto 1,1
	#  MoveTo 2,2
	#  LineTo 3,2.2
	assert testMoves([edge1,edge3]) == 4,"Should be four moves"
	print "[OK]"
	
	print "Arcs And Lines"
	wire = makeTestWire();
	#the correct answer is:
	#  Moveto 40,50
	#  ArcTo  50,40 with center 40,40, ccw direction
	#  LineTo 80,40
	assert testMoves([wire]) == 3,"Should be 3 moves?"
	print "[OK]"
	
	print "Arcs and Lines-- No Arcs"
	# should MoveTo 40,50
	# draw to 50,40 in a lot of little moves
	# draw to 80,40 in one move
	assert testMoves([wire],False) > 10,"Should be lots of moves"
	
	print "Square Wire"
	assert testMoves([TestDisplay.makeSquareWire()],False) > 3;
	