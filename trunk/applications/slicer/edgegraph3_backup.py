"""
	EdgeGraphs
	
	Utilties for representing wires and lists of edges in a graph.
	
"""

from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
import time,os,sys,string;
from OCC.Geom import *
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();
import TestDisplay
import itertools
import Wrappers
import time
import networkx as nx
import cProfile
import pstats

def tP(point):
	"return a tuple for a point"
	return (point.X(), point.Y() );
	
def hashE(edge):
	return edge.HashCode(1000000);

	
	
class PointOnAnEdge:
	"stores an intersection point"
	def __init__(self,edge,param,point):
		self.edge = edge;
		self.param = param;
		self.point = point;

"""
	Graph of edges.
	Also provides finding a single edge in the chain,
	navigating along the chain in both directions,
	and navigating from the starting point of a particular edge.
	
	This is a bit different than a normal graph, it contains extra 
	functions to find edges using their native OCC edge object and parameter.
	
"""
class EdgeGraph:

	def __init__ (self):
		"make an empty edgegraph"
	
		"""
			dictionary contains each underlying edge, and all of 
			the nodes lying on that edge. multiple edge nodes
			can lie on a single edge because edge nodes might subdivide an edge
			
			Nodes in this graph are actually OCC edges.
			
			One major complexity comes from the fact that multiple nodes can share
			the same underlying edge-- this happens when a single edge is subdivided.
			the only difference is the parameters on the edge in that case.
			
		"""
		# key=edgeHash: value=dict(key=nodehash,value=node)
		self.edges = {}; #store nodes organized by hashed edge and parameter. dict of dicts

		#graph for the edges and nodes. nodes are 2d tuples ( x,y)
		#edges are nx edges with EdgeSegment objects in the attributes
		self.g = nx.Graph();
	
	def firstNode(self):
		return self.edges.values()[0].values()[0];

	def divideEdge(self,edge,param):
		"""
			split the edge at the provided parameter,
			creating two new edge nodes.
			update internal structures accordingly.
		"""
		original = self.findEdge(edge,param);
		
		ew = Wrappers.Edge(edge);
		
		if original == None:
			raise ValueError,"Could not find edge having parameter %0.3f" % (param);
			
		#param must be between the bounds on the original edge
		assert param >= original.p1 and param <= original.p2;
		
		#compute nodes on either end.
		n1 = tP(original.ew.firstPoint);
		n2 = tP(original.ew.lastPoint);
		
		#add new node and edges
		n3 = tP(ew.pointAtParameter(param));
		newNode1 = EdgeSegment(ew,original.p1,param,original.type);
		newNode2 = EdgeSegment(ew,param,original.p2, original.type );

		self.addEdgeSegment(newNode1);
		self.addEdgeSegment(newNode2);
		
		#delete the original
		self.removeEdge(original);
				
		return [newNode1,newNode2];
		
	def findEdge(self,edge,param):
		"""
			Find the edge having the parameter of
			the contained value. returns none if not found.
			typically used to find an intersection value, 
		"""
		
		#TODO: it would be good to find a faster way to 
		#do this,ie, to hash by parameter or sort instead
		#of looping through all
		
		for n in self.edges[hashE(edge)].values():
			if param >= n.p1 and param <= n.p2:
				return n;
			#else:
			#	print "checking, param=%0.3f,p1=%0.3f,p2=%03.f" % ( param, n.p1, n.p2 );
		return None;


	def addEdgeSegment(self,edgeSegment):
		"adds an EdgeSegment to the structure"
		#adds the nodes and the edges all in one shot to the graph
		self.g.add_edge(tP(edgeSegment.firstPoint),tP(edgeSegment.lastPoint),{ 'node':edgeSegment} );
		
		#store the edge in a dict by edge hash.
		#the key of the second dict is hash+p1+p2-- ie, each distint edge, and paramter pair are stored
		eh = hashE(edgeSegment.edge);
		if not self.edges.has_key(eh):
			self.edges[eh] = {};		
		l = self.edges[eh];
		
		l[edgeSegment] = edgeSegment;
		
	def addEdge(self,edge,type):
		"adds an edgeto the structure, using the endpoints of the edge as the nodes"
		"this is equivalent to adding an edgesegment that spans the full edge"
		ew = Wrappers.Edge(edge);
		
		newEdge = EdgeSegment(ew,ew.firstParameter,ew.lastParameter,type);
		self.addEdgeSegment(newEdge);

	def removeEdge(self,edgeSegment):
		"remove an EdgeSegment from the structure"
		
		#remove from hash to find by original edge
		e = edgeSegment.edge;
		d =  self.edges[hashE(e)];
		d.pop(edgeSegment);
		
		#remove edge from nested graph
		n1 = tP(edgeSegment.firstPoint);
		n2 = tP(edgeSegment.lastPoint);
		self.g.remove_edge(n1,n2);
		
	def addWire(self,wire,type):
		"add all the edges of a wire. They will be connected together."
		"each edge is added at its full length"
		
		#for finding an edge node
		wr = Wrappers.Wire(wire);
		eS = wr.edgesAsSequence();

		for i in range(1,eS.Length()+1):
			e = Wrappers.cast(eS.Value(i));
			self.addEdge(e,type);

		#link last and first edge if the wire is closed
		#if wire.Closed():
		#	firstEdge = eS.Value(1);
		#	lastEdge = eS.Value(eS.Length());
		#	
		#	self.linkPrev( firstNode,lastNode);
		#	self.linkNext( lastNode,firstNode);			

	def allEdgesRandomOrder(self):
		"return all the edges"
		for e in self.g.edges_iter():
			#wow, python perl-like hell
			#each edge is a tuple of nodes.
			#then we have to extract the edge dict and get the node value
			yield self.g[e[0]][e[1]]['node'];
	

"""
	use intersection points along a wire
	to divide it into individual pieces
	using scanline algorithm.
	
	The result is a list of edges that should be included.
	No effort is made to assemble these edges into wires. the
	result could be a disjointed set of wires.
"""
def splitWire(wire, ipoints):
	"""
		ipoints is a list of intersection points.
	"""
	
	#load original wire
	#we make an important assumption that the wire is organized from
	#left to right or from right to left, and starts outside the boundaries.
	#this allows the scanline algorithm to define which segments are 'inside'
	wr = Wrappers.Wire(wire);
	
	#assume edges are in ascending x order also
	eS = wr.edgesAsList();

	#sort intersection points by ascending X location
	ix = sorted(ipoints,key = lambda p: p.point.X() );
	
	inside = False;		
	edges = []; #a list of edges for the current wire.	
	iEdge = 0;
	iIntersection=0;
	
	#TODO: handle odd number of intersections
	
	#the last parameter on the current edge.
	#it is either the first parameter on the current edge,
	#or the last intersection point on the edge.
	currentEdge = eS[iEdge];
	
	#currentEdgeWrapper = Wrappers.Edge(currentEdge);
	currentEdgeBounds = brepTool.Range(currentEdge);
	
	startParam = currentEdgeBounds[0];
	
	while iIntersection < len(ix) and ( iEdge < len(eS)  ) :
		currentIntersection = ix[iIntersection];

		if hashE(currentEdge) == hashE(currentIntersection.edge):
						
			#current edge matches intersection point
			if inside:
				#transition to outside: add this part edge
				#currentEdgeWrapper = Wrappers.Edge(currentEdge);
				currentEdgeBounds = brepTool.Range(currentEdge);
				#startParam = currentEdgeWrapper.firstParameter;
				newEdge = Wrappers.trimmedEdge(currentEdge,startParam,currentIntersection.param);
				#newEdge = currentEdgeWrapper.trimmedEdge(startParam, currentIntersection.param);
				edges.append(newEdge);
							
			#move to next point, store last intersection
			iIntersection += 1;
			startParam = currentIntersection.param;			
			inside = not inside;
			
		else:
			#edges do not match		
			if inside:
				currentEdgeBounds = brepTool.Range(currentEdge);
				#currentEdgeWrapper = Wrappers.Edge(currentEdge);
				if startParam == currentEdgeBounds[0]:
					edges.append(currentEdge);
				else:
					newEdge = Wrappers.trimmedEdge(currentEdge,startParam, currentEdgeBounds[1] );
					#newEdge = currentEdgeWrapper.trimmedEdge(startParam, currentEdgeWrapper.lastParameter);
					edges.append(newEdge);
			
			#move to next edge
			iEdge += 1;
			currentEdge = eS[iEdge];
			#currentEdgeWrapper = Wrappers.Edge(currentEdge);
			startParam = currentEdgeBounds[0];			

	#done. return the edges
	return edges;

	
"an edge in a connected set of edges."
class EdgeSegment:
	"a sub-portion of a parameterized edge"
	def __init__(self,edgeWrapper,p1,p2,type):
		"edge is a ref to an edge, with the beginning and ending parameters"
		"for the underlying curve"
		self.edge = edgeWrapper.edge;
		self.ew = edgeWrapper;
		self.p1 = p1;
		self.p2 = p2;
		self.type =type;
		self.firstPoint = self.ew.pointAtParameter(self.p1);
		self.lastPoint = self.ew.pointAtParameter(self.p2);
		
	def newEdge(self):
		"make a new edge between specified parameters"
		return self.ew.trimmedEdge(self.p1, self.p2 );
		
	"define these methods so nodes just magically work in dicts"
	def __eq__(self,p1):
		return self.key().__eq__(p1);
		
	def __hash__(self):
		return self.key().__hash__();
		
	def key(self):
		"""
			since multiple EdgeSegments can be associated with a single edge,
			the key is a tuple consisting of the edge hash the the start parameter on the edge
		"""
		return ( hashE(self.edge),self.p1,self.p2);

	def __repr__(self):
		return  "EdgeSegment: %d ( %0.3f - %0.3f )" % ( hashE(self.edge),self.p1, self.p2);

			


def testSplitWire1():
	"""
		Test split wire function. there are two main cases:
		wires with intersection on different edges,
		and a wire with a single edge split in many places
	"""
		
	#case 1: a single edge with lots of intersections along its length
	e  = Wrappers.edgeFromTwoPoints( gp.gp_Pnt(0,0,0),gp.gp_Pnt(5,0,0));
	w = Wrappers.wireFromEdges([e]);
	
	#out of order on purpose
	p1 = PointOnAnEdge(e,1.0,gp.gp_Pnt(1.0,0,0));
	p3 = PointOnAnEdge(e,3.0,gp.gp_Pnt(3.0,0,0));
	p2 = PointOnAnEdge(e,2.0,gp.gp_Pnt(2.0,0,0));	
	p4 = PointOnAnEdge(e,4.0,gp.gp_Pnt(4.0,0,0));
	ee = splitWire(w,[p1,p3,p2,p4] );

	assert len(ee) ==  2;
	length = 0;
	for e in ee:
		ew = Wrappers.Edge(e);
		length += ew.distanceBetweenEnds();
		TestDisplay.display.showShape(e);

	assert length == 2.0;
	
def testSplitWire2():
	"intersections on different edges. one edge completely inside"
	
	e1 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(0,0,0),gp.gp_Pnt(2,0,0));
	e2 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(2,0,0),gp.gp_Pnt(5,0,0));
	e3 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(5,0,0),gp.gp_Pnt(6,0,0));
	
	
	#trick here. after building a wire, the edges change identity.
	#evidently, BRepBuilder_MakeWire makes copies of the underliying edges.
	
	w = Wrappers.wireFromEdges([e1,e2,e3]);
	ee = Wrappers.Wire(w).edgesAsList();
	print "Original Edges: %d %d %d " % ( hashE(ee[0]),hashE(ee[1]),hashE(ee[2]));
	p1 = PointOnAnEdge(ee[0],1.0,gp.gp_Pnt(1.0,0,0));
	p2 = PointOnAnEdge(ee[2],0.5,gp.gp_Pnt(5.0,0,0));
	
	
	ee = splitWire(w,[p2,p1]);
	
	assert len(ee) == 3;
	
	length = 0;
	for e in ee:
		ew = Wrappers.Edge(e);
		length += ew.distanceBetweenEnds();
		TestDisplay.display.showShape(e);
	print "length=%0.3f" % length;
	assert length == 4.5;

	
def testDivideWire():
	w = TestDisplay.makeCircleWire();
	#w = TestDisplay.makeSquareWire();
	#w = TestDisplay.makeReversedWire();
	
	wr = Wrappers.Wire(w);
	
	eg = EdgeGraph();
	eg.addWire(w,'BOUND');
	print eg.edges
	
	#the big test-- split one of the edges
	e = eg.firstNode().edge;

	[newEdge1,newEdge2] = eg.divideEdge(e,2.5);
	print eg.edges
	e2 = newEdge2.edge;
	[newEdge3,newEdge4] = eg.divideEdge(e2,0.2333 );

	for en in eg.allEdgesRandomOrder():
		time.sleep(.1);
		e = en.newEdge();
		TestDisplay.display.showShape( TestDisplay.makeEdgeIndicator(e) );
		TestDisplay.display.showShape(e );	

		
def splitPerfTest():
	"make a long wire of lots of edges"
	"""
		performance of the wire split routine is surprisingly bad!
		
	"""
	
	WIDTH=0.1
	edges = [];
	for i in range(1,50):
		e = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(i*WIDTH,0,0),gp.gp_Pnt((i+1)*WIDTH,0,0))
		TestDisplay.display.showShape(e);
		edges.append(e);
		
	
	#trick here. after building a wire, the edges change identity.
	#evidently, BRepBuilder_MakeWire makes copies of the underliying edges.
	
	w = Wrappers.wireFromEdges(edges);
	ee = Wrappers.Wire(w).edgesAsList();
	
	#compute two intersections
	e1 = Wrappers.Edge(ee[5]);
	e2 = Wrappers.Edge(ee[30]);
	e1p = (e1.lastParameter - e1.firstParameter )/ 2;
	e2p = (e2.lastParameter - e2.firstParameter )/ 2;
	p1 = PointOnAnEdge(e1.edge,e1p ,e1.pointAtParameter(e1p));
	p2 = PointOnAnEdge(e2.edge,e2p ,e2.pointAtParameter(e2p));
	
	#cProfile.runctx('for i in range(1,100): ee=splitWire(w,[p2,p1])', globals(), locals(), filename="slicer.prof")
	#p = pstats.Stats('slicer.prof')
	#p.sort_stats('time')
	#p.print_stats(.98);		

	t = Wrappers.Timer();
	for i in range(1,100):
		ee = splitWire(w,[p2,p1]);
	print "Elapsed for 100 splits:",t.finishedString();
	
	
def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('tot')
	p.print_stats(level);	
	
if __name__=='__main__':
	print "Basic Wrappers and Utilities Module"

	e1 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(0,0,0),gp.gp_Pnt(2,0,0));
	
	"""
	t = Wrappers.Timer();
	for i in range(1,20000):
		ew = Wrappers.Edge(e1);
	print "Create 10000 edgewrappers= %0.3f" % t.elapsed();
	print ew.firstParameter, ew.lastParameter;
	
	t = Wrappers.Timer();
	for i in range(1,20000):
		(s,e) = brepTool.Range(e1);
	print "Get parameters 10000 times= %0.3f" % t.elapsed();
	print s,e
	"""
	
	#testDivideWire();
	print "Testing One Edge Lots of parms.."
	testSplitWire1();
	
	print "Testing lots of edges"
	testSplitWire2();
	splitPerfTest();
	print "Tests Complete.";
	
	TestDisplay.display.run();			
		