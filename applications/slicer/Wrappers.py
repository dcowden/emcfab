"""
 Defines some useful wrappers and utility functions
 Most of these make dealing with underlying APIs require less code,
 and behave more pythonically
 
"""
from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
import time,os,sys,string;
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();
import TestDisplay
import itertools

#topexp = TopExp.TopExp()
#texp = TopExp.TopExp_Explorer();
#brt = BRepTools.BRepTools();
#log = logging.getLogger('Wrapper');

"""
	Utility class to provide timing information
"""		
class Timer:
	def __init__(self):
		self.startTime = time.time();
		self.startAscTime = time.asctime();
		
	def start(self):
		return self.startTime;
			
	def elapsed(self):
		end = time.time();
		return end - self.startTime;
		
	def finishedString(self):
		return "%0.3f sec" % ( self.elapsed() );


def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return itertools.izip(a, b)
	
def ntuples(lst, n,wrap=True):
	B = 0;
	if not wrap:
		B = 1;
	return zip(*[lst[i:]+lst[:(i-B)] for i in range(n)])
	
"""
	A floating point range generator
"""		
def frange6(*args):
    """A float range generator."""
    start = 0.0
    step = 1.0

    l = len(args)
    if l == 1:
        end = args[0]
    elif l == 2:
        start, end = args
    elif l == 3:
        start, end, step = args
        if step == 0.0:
            raise ValueError, "step must not be zero"
    else:
        raise TypeError, "frange expects 1-3 arguments, got %d" % l

    v = start
    while True:
        if (step > 0 and v >= end) or (step < 0 and v <= end):
            raise StopIteration
        yield v
        v += step	

def make_vertex(pnt):
	"make a vertex from a single point"
	s = BRepBuilderAPI.BRepBuilderAPI_MakeVertex(pnt);
	s.Build();
	return s.Shape();

def listFromHSequenceOfShape(seq):
	"return a list from a sequence o shape"
	newList = [];
	for i in range(1,seq.Length()+1):
		newList.append(cast(seq.Value(i)));
	return newList;

def hSeqIterator(hSeq):
	"sane iteration over hsequenceof shape"
	for i in range(1,hSeq.Length()+1):
		yield cast(hSeq.Value(i));
		
def extendhSeq(hSeqTo, hSeqFrom):
	"append elements in from to to"
	for e in hSeqIterator(hSeqFrom):
		hSeqTo.Append(e);

def edgeFromTwoPoints(p1,p2):
	"make a linear edge from two points "
	builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2);
	builder.Build();
	if builder.IsDone():
		return builder.Edge();
	else:
		return None;
		
def findParameterOnCurve(point,handleCurve,tolerance):
	"return the closest parameter on the curve for the provided point"
	"returns none if a point is not within tolerance"
	#TODO: Extrema.Extrema_ExtPC will project a point onto an Adaptor_3d Curve
	#so we do not have to approximate with a bspline
	#BrepAdaptor.BrepAdaptor_CompCurve.Trim() will return a trimmed curve between two parameters
	#E1CLib can project points on elementary curves (lines and conics )
	gp = GeomAPI.GeomAPI_ProjectPointOnCurve(point,handleCurve);
	gp.Perform(point);
	if gp.NbPoints()>0:
		#log.debug( "Projection Success!" );
		return [point, gp.LowerDistanceParameter(),gp.LowerDistance()];
	else:
		#log.error( "Projection Failed.");
		return None;

		
def findPointOnCurve(point,handleCurve,tolerance):
	"return the closest parameter on the curve for the provided point"
	"returns none if a point is not within tolerance"
	gp = GeomAPI.GeomAPI_ProjectPointOnCurve(point,handleCurve);
	gp.Perform(point);
	if gp.NbPoints()>0 and gp.LowerDistance() <= tolerance:
		#log.debug( "Projection Success!" );
		return [gp.LowerDistanceParameter(),gp.NearestPoint()];
	else:
		#log.warn( "Projection Failed.");
		return None;

def makeWiresFromOffsetShape(shape):
	"get all the wires from the offset shape"
	resultWires = TopTools.TopTools_HSequenceOfShape();
	if shape.ShapeType() == TopAbs.TopAbs_WIRE:
		#log.info( "offset result is a wire" );
		wire = topoDS.Wire(shape);
		#resultWires.append(wire);
		resultWires.Append(wire);
	elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
		#log.info( "offset result is a compound");

		bb = TopExp.TopExp_Explorer();
		bb.Init(shape,TopAbs.TopAbs_WIRE);
		while bb.More():
			w = topoDS.Wire(bb.Current());
						
			#resultWires.append(w);
			resultWires.Append(w);#
			#debugShape(w);
			bb.Next();
		
		bb.ReInit();	
	
	return resultWires;	

def wireFromEdges(edgeList):
	"TODO: might need to use sortwires to make sure it is i nthe right order"
	mw = BRepBuilderAPI.BRepBuilderAPI_MakeWire();
	for e in edgeList:
		mw.Add(e);
		
	if mw.IsDone():
		return mw.Wire();
	else:
		raise ValueError,"Error %d Building Wire" % mw.Error();

def pointAtParameterList(edge,paramList):
	"compute the point on the edge at the supplied parameter"
	hc = brepTool.Curve(edge);
	c = GeomAdaptor.GeomAdaptor_Curve(hc[0]);
	r = [];
	for p in paramList:
		r.append(c.Value(p));
	return r;

		

def pointAtParameter(edge,param):
	hc = brepTool.Curve(edge);
	c = GeomAdaptor.GeomAdaptor_Curve(hc[0]);
	return c.Value(param);

def trimmedEdge(edge,p1, p2):
	"returns a new edge that is a trimmed version of the underlying one"
	hc = BRep.BRep_Tool().Curve(edge);
	return edgeFromTwoPointsOnCurve(hc[0],p1,p2);
	
def edgeFromTwoPointsOnCurve(handleCurve,p1,p2):
	"make an edge from a curve and two parameters"
	#log.info("Making Edge from Parameters %0.2f, %0.2f" %( p1, p2) );
	
	a = p1;
	b = p2;
	rev = False;
	if p1 > p2:
		a = p2;
		b = p1;
		#log.info("Parameters are inverted, so we'll reverse the resulting edge");
		rev = True;		
	
	builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(handleCurve,a,b);
	builder.Build();
	if builder.IsDone():
		e = builder.Edge();
		if rev:
			e.Reverse();
		return e;
	else:
		#log.error( "Error building Edge: Error Number %d" % builder.Error());
		raise ValueError,"Error building Edge: Error Number %d" % builder.Error();
		
def cast(shape):
	"these names have changed between occ 0.4 and occ 0.5!!!"

	type = shape.ShapeType();
	"cast a shape as its correct type"
	if type == TopAbs.TopAbs_WIRE:
		return topoDS.wire(shape);
	elif type == TopAbs.TopAbs_EDGE:
		return topoDS.edge(shape);
	elif type == TopAbs.TopAbs_VERTEX:
		return topoDS.vertex(shape);
	elif type == TopAbs.TopAbs_SOLID:
		return topoDS.solid(shape);
	elif type == TopAbs.TopAbs_FACE:
		return topoDS.face(shape);
	elif type == TopAbs.TopAbs_SHELL:
		return topoDS.shell(shape);
	elif type == TopAbs.TopAbs_COMPOUND:
		return topoDS.compound(shape);
	return shape;


"""
	Point
"""
class Point:
	def __init__(self,point):
		self.point = point;
		
	def __str__(self):
		return "[%0.3f %0.3f %0.3f]" % (self.point.X(), self.point.Y(), self.point.Z());
		
"""
  Edge: provides useful functions for dealing with an edge shape
"""
class Edge:
	def __init__(self,edge):

		self.edge = edge;		
		hc = BRep.BRep_Tool().Curve(edge);
		self.curve = GeomAdaptor.GeomAdaptor_Curve(hc[0]); 
		self.handleCurve  = hc[0];
		self.firstParameter = hc[1];
		self.lastParameter = hc[2];

		p1 = self.curve.Value(self.firstParameter);
		p2 = self.curve.Value(self.lastParameter);

		#compute the first and last points, which are very commonly used		
		if edge.Orientation() == TopAbs.TopAbs_FORWARD:
			self.reversed = False;
			self.firstPoint = p1;
			self.lastPoint = p2;
		else:
			self.reversed = True;
			self.firstPoint = p2;
			self.lastPoint = p1;
	
	def distanceBetweenEnds(self):
		"length of the edge, assuming it is a straight line"
		return self.firstPoint.Distance(self.lastPoint);
		
	def isLine(self):
		return self.curve.GetType() == GeomAbs.GeomAbs_Line;
	
	def pointAtParameter(self,p):
		"return the point corresponding to the desired parameter"
		if p < self.firstParameter or p > self.lastParameter:
			raise ValueError,"Expected parameter between first and last."
		return self.curve.Value(p);

	def splitAtParameter(self,p):
		"""
		  split this edge at the supplied parameter.
		  returns two EdgeWwrappers
		"""
		e1 = self.trimmedEdge(self.firstParameter,p);
		e2 = self.trimmedEdge(p,self.lastParameter);
		return [e1,e2];
	
	def trimmedEdge(self,parameter1, parameter2):
		"make a trimmed edge based on a start and end parameter on the same curve"
		"p1 and p2 are either points or parameters"

		e = edgeFromTwoPointsOnCurve(self.handleCurve, parameter1, parameter2 );		
		return e;
		
	def isCircle(self):
		return self.curve.GetType() == GeomAbs.GeomAbs_Circle;
	
	def discretePoints(self,deflection):
		"a generator for all the points on the edge, in forward order"
		gc = GCPnts.GCPnts_QuasiUniformDeflection(self.curve,deflection,self.firstParameter,self.lastParameter);
		print "Making Dicrete Points!"
		i = 1;
		numPts = gc.NbPoints();
		while i<=numPts:
			if self.reversed:
				yield gc.Value(numPts-i+1);
			else:
				yield gc.Value(i);
			i+=1;
			
	def __str__(self):
		if self.isLine():
			s = "Line:\t";
		elif self.isCircle():
			s  = "Circle:\t";
		else:
			s = "Curve:\t"; 
		return s + str(Point(self.firstPoint)) + "-->" + str(Point(self.lastPoint));


"""
	Follow a wire's edges in order
"""
class Wire():
	def __init__(self,wire):
		self.wire = wire;
		
	def edges2(self):
		"return edges in a list"
		wireExp = BRepTools.BRepTools_WireExplorer(self.wire);
		while  wireExp.More():
			e = wireExp.Current();
			yield e;
			wireExp.Next();
		#wireExp.Clear();
	
	def edgesAsSequence(self):
		"returns edge list as a sequence"
		wireExp = BRepTools.BRepTools_WireExplorer(self.wire);
		resultWires = TopTools.TopTools_HSequenceOfShape();
		
		while  wireExp.More():
			e = wireExp.Current();
			resultWires.Append( e);
			wireExp.Next();
		
		return resultWires;
		
	def edgesAsList(self):
		edges = [];
		for e in self.edges2():
			edges.append(e);
		return edges;

	def edges(self):
		"a generator for edges"
		bb = TopExp.TopExp_Explorer();
		bb.Init(self.wire,TopAbs.TopAbs_EDGE);
		while bb.More():
			e = topoDS.edge(bb.Current());
			yield e;	
			bb.Next();		
		bb.ReInit();		
		
	def assertHeadToTail(self):
		"checks that a wire is head to tail."
		#log.info("Asserting Wire..");
		seq = self.edgesAsSequence();
		list  = listFromHSequenceOfShape(seq);
		wrapAround = self.wire.Closed();
		#if wrapAround:
			#log.info("Wire is closed.");
		for (edge1,edge2) in ntuples(list,2,wrapAround):
			ew1 = Edge(edge1);
			ew2 = Edge(edge2);
			assert ew1.lastPoint.Distance(ew2.firstPoint) < 0.0001;
		#log.info("Wire is valid.");
		
	def __str__(self):
		"print the points in a wire"
		s = ["Wire:"];
		for edge in self.edges2():
			ew = Edge(edge);
			s.append( str(ew) );
		return "\n".join(s);
		
		
	def discretePoints(self,deflection):
		"discrete points for all of a wire"
		for e in self.edges():
			for m in Edge(e).discretePoints(deflection):
				yield m;
		
if __name__=='__main__':
	print "Basic Wrappers and Utilities Module"

	w = TestDisplay.makeSquareWire();
	
	el = EdgeLinkedList(w);

	#TestDisplay.display.showShape(w);
	#TestDisplay.display.showShape(el.first().edge);
	#TestDisplay.display.showShape(el.last().edge);
	
	#for e in el.nodesForward():
	#	TestDisplay.display.showShape(e.edge);
	
	for e in el.nodesBackward():
		TestDisplay.display.showShape(e.edge);
		
	TestDisplay.display.run();
	

