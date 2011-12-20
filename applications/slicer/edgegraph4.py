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
from OCC.Utils.Topology import Topo
from OCC.Utils.Topology import WireExplorer

import TestDisplay
import itertools
import Wrappers
import time
import networkx as nx
import cProfile
import pstats
import hexagonlib
import traverse

def tP(point,places=3):
	"""
	    return a tuple for a point.
		rounds to the number of places specified to ensure that
		rounding error does not result in non-coincident nodes
	"""
	return (round(point.X(),places), round(point.Y(),places) );
	
	
class PointOnAnEdge:
	"stores an intersection point"
	def __init__(self,edge,param,point):
		self.edge = edge;
		self.hash = self.edge.__hash__();
		self.param = param;
		self.point = point;

class 
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
		self.fillEdges = []; #ordered list of edges-- used when we walk the graph later.
		
		#graph for the edges and nodes. nodes are 2d tuples ( x,y)
		#edges are nx edges with EdgeSegment objects in the attributes
		self.g = nx.Graph();
		self.startNode = False

	def getEdge(self,n1,n2):
		return self.g.get_edge_data(n1,n2);
		
	def divideEdge(self,edge,param):
		"""
			split the edge at the provided parameter,
			creating two new edge nodes.
			update internal structures accordingly.
		"""
		original = self.findEdge(edge,param);
		
		if original == None:
			raise ValueError,"Could not find edge having parameter %0.3f" % (param);
			
		#param must be between the bounds on the original edge
		assert param >= original.p1 and param <= original.p2;
		
		#compute nodes on either end.
		n1 = tP(original.firstPoint);
		n2 = tP(original.lastPoint);
		
		#add new node and edges
		#n3 = tP( Wrappers.pointAtParameter(edge,param));
		newNode1 = EdgeSegment(edge,original.p1,param,original.type);
		newNode2 = EdgeSegment(edge,param,original.p2, original.type );

		
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
		
		for n in self.edges[edge.__hash__()].values():
			if param >= n.p1 and param <= n.p2:
				return n;
			#else:
			#	print "checking, param=%0.3f,p1=%0.3f,p2=%03.f" % ( param, n.p1, n.p2 );
		return None;


	def addEdgeSegment(self,edgeSegment):
		"adds an EdgeSegment to the structure"
		#adds the nodes and the edges all in one shot to the graph
		#print "Adding Edge :%d " % edgeSegment.edge.__hash__();
		#TestDisplay.display.showShape(edgeSegment.edge);
		
		#hack-- easy way to store the first node is to wathc for the first fill type node added.
		if self.startNode == False and edgeSegment.type == 'FILL':
			self.startNode = tP(edgeSegment.firstPoint);
		p1 = tP(edgeSegment.firstPoint);
		p2 = tP(edgeSegment.lastPoint);
		self.g.add_edge(p1,p2,{'node': edgeSegment, 'type': edgeSegment.type} );
		
		#keep track of fill edges in the order they were added
		if edgeSegment.type == 'FILL':
			self.fillEdges.append( (p1,p2) );
			
		#store the edge in a dict by edge hash.
		#the key of the second dict is hash+p1+p2-- ie, each distint edge, and paramter pair are stored
		eh = edgeSegment.hash;
		l = self.edges.setdefault(eh,{});
		#if not self.edges.has_key(eh):
		#	self.edges[eh] = {};		
		#l = self.edges[eh];

		l[edgeSegment] = edgeSegment;
		
	def addEdge(self,edge,etype):
		"adds an edgeto the structure, using the endpoints of the edge as the nodes"
		"this is equivalent to adding an edgesegment that spans the full edge"
		(f,l) = brepTool.Range(edge);
		newEdge = EdgeSegment(edge,f,l,etype);
		self.addEdgeSegment(newEdge);
		#print "Adding %s Edge (%0.3f, %0.3f ) <--> (%0.3f %0.3f ) " % ( type, newEdge.firstPoint.X(), newEdge.firstPoint.Y(), newEdge.lastPoint.X(), newEdge.lastPoint.Y()  ) ;


	def removeEdge(self,edgeSegment):
		"remove an EdgeSegment from the structure"
		#print "Removing Edge %s " % str(edgeSegment.key());
		#remove from hash to find by original edge
		e = edgeSegment.edge;
		d =  self.edges[e.__hash__()];
		#print edgeSegment.type 
		d.pop(edgeSegment);
		
		assert edgeSegment.type == 'BOUND'
			
		#remove edge from nested graph
		n1 = tP(edgeSegment.firstPoint);
		n2 = tP(edgeSegment.lastPoint);
		try:
			self.g.remove_edge(n1,n2);
		except:
			pass;
		#print "Removing %s Edge (%0.3f, %0.3f ) <--> (%0.3f %0.3f ) " % ( edgeSegment.type, edgeSegment.firstPoint.X(), edgeSegment.firstPoint.Y(), edgeSegment.lastPoint.X(), edgeSegment.lastPoint.Y()  ) ;
	
	def addEdgeListAsSingleEdge(self,edgeList,etype):
		"""
			adds a list of edges in the graph as a single edge.
			The list of edges becomes a 'pseudo-edge' with a single start and endpoint.
			The edge cannot be divided, and its vertices are not stored in the edge graph
		"""
		
		#TODO: adding this method has broken the EdgeSegment intervace.
		#what is needed is an object that exposes first and last point, and a container for the underlying object
		# an edgesegment ( a portion of an edge between two parameters ), a full edge ( requring no trimming ), 
		# and a gropu of edges combined are all special subcases.
		
		firstEdge = edgeList[0];
		lastEdge = edgeList[-1];
		(f,l) = brepTool.Range(firstEdge);
		(n,m) = brepTool.Range(lastEdge);
		
		#todo, this can be optimized, the underlying curve is computed twice here.
		#something very odd is happening here. whey does using the last parameter and first paraterm
		#work ? initially i had them backewards, which should have worked, but opposites worked.
		#wierd. do i need to account for the sense?
		#maybe i should just use wrappers.edge here
		if firstEdge.Orientation() == TopAbs.TopAbs_FORWARD:
			p1 = tP( Wrappers.pointAtParameter( firstEdge,f));
		else:
			p1 = tP( Wrappers.pointAtParameter ( firstEdge,l));
		
		if lastEdge.Orientation() == TopAbs.TopAbs_FORWARD:	
			p2 = tP(Wrappers.pointAtParameter(lastEdge,m ));
		else:
			p2 = tP(Wrappers.pointAtParameter(lastEdge,n )); 
		
		#todo, i dont think we care about this key do we, as long as it is unique?
		self.g.add_edge(p1,p2,{'edgeList': edgeList,"type":etype} );
		if etype == 'FILL':
			self.fillEdges.append( ( p1,p2 ));
		
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

	def allNodesRandomOrder(self):
		for n in self.g.nodes_iter():
			yield n;
			
	def allEdgesRandomOrder(self):
		"return all the edges"

		for e in self.g.edges_iter(data=True):
			#wow, python perl-like hell
			#each edge is a tuple of nodes.
			#then we have to extract the edge dict and get the node value
			#print e[2].keys();#
			yield e[2];

	def walkEdges(self):
		"return a path that will walk all infill edges in the graph"
		edgeList = traverse.travelAll(self.g,self.fillEdges,isFillEdge );
		return edgeList;

def isFillEdge(graph,n1,n2):
	return graph[n1][n2]['type'] == 'FILL';
	
def splitWire(wire,ipoints):
	"""
		ipoints is a list of intersection points.
		returns a list of wires inside the intersection point
		
		this method must also work for a 'wire' consisting of a single
		edge. more than one intersection point can be on each edge, 
		but all of the ipoints are expected to be on edges in the provided wire.
		BASELINE PERFORMANCE: 11 ms per call for splitwiretest
		
		returns a list of lists.
		each element in the top list is a chain of connected edges.
			each element in that list is an edge ( or part of an edge )
			
		so, for example, suppose you compute a wire that has 100 edges, and there are 4 intersection points.
		in this case, there will be two elements returned, and each element would contain a list of edges
		between the two segments.
		---E1---+---E2---+-X-E3---+---E4--X--+---E5---
		will return [  [E1,E2], [ E3,E4 ] ]
		
	"""
	topexp = TopExp.TopExp_Explorer();
	topexp.Init(wire,TopAbs.TopAbs_EDGE);

	#sort intersection points by ascending X location
	ix = sorted(ipoints,key = lambda p: p.point.X() );	

	edgeList = [];
	
	assert (len(ipoints) % 2) == 0;
	
	for i in range(0,len(ipoints),2):
		#process intersection points in pairs
		#TODO: this could be done more cleanly with a pairwise iterator
		currentIntersection = ix[i];
		nextIntersection = ix[i+1];
		
		#if they are on the same edge, simply add a trimmed edge.
		#in this case, 
		if currentIntersection.hash == nextIntersection.hash:
			edgeList.append ( [ Wrappers.trimmedEdge(currentIntersection.edge, currentIntersection.param, nextIntersection.param ) ] ); 
		else:
			#intersections are not on the same edge.
			#add the fraction of the first edge
			(bp,ep) = brepTool.Range(currentIntersection.edge);
			edges = [];
			#print "adding piece of start edge."
			edges.append( Wrappers.trimmedEdge(currentIntersection.edge,currentIntersection.param, ep));
	
			#pick up whole edges between the first intersection and the next one
			#loop till current edge is same as current intersection
			while topexp.Current().__hash__() != currentIntersection.hash:
				topexp.Next();
			
			#advance to next edge
			topexp.Next();
			
			#add edges till current edge is same as next intersection
			#most of the performance suckage is happening here, with gc associated with these
			#edge objects.  If that gets fixed, we'll get a huge speed boost. about 33% of total time is saved.
			while topexp.Current().__hash__() != nextIntersection.hash:
				edge = Wrappers.cast(topexp.Current() );
				edges.append(edge);
				#print "adding middle edge"
				topexp.Next();
	
			#add the last edge
			(bp,ep) = brepTool.Range(nextIntersection.edge);
			edges.append( Wrappers.trimmedEdge(nextIntersection.edge,bp,nextIntersection.param));
			#print "adding last piece of edge"
			edgeList.append(edges);
	return edgeList;

	
"an edge in a connected set of edges."
class EdgeSegment:
	"one or more sub-portions of an edge"
	def __init__(self,edge, edgetype):
		self.edge = edge;
		self.points = []; #list of point on edge objects
		self.edgetype =edgetype;
	
	def addPoint(self,param,point):
		"adds a point for this edge"

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
		ew = Wrappers.Edge(e[0]);
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
	#print "Original Edges: %d %d %d " % ( ee[0].__hash__(),ee[1].__hash__(),ee[2].__hash__());
	p1 = PointOnAnEdge(ee[0],1.0,gp.gp_Pnt(1.0,0,0));
	p2 = PointOnAnEdge(ee[2],0.5,gp.gp_Pnt(5.0,0,0));
	
	
	ee = splitWire(w,[p2,p1]);
	
	assert len(ee) == 1;
	
	length = 0;
	for e in ee[0]:
		ew = Wrappers.Edge(e);
		length += ew.distanceBetweenEnds();
		TestDisplay.display.showShape(e);
	#print "length=%0.3f" % length;
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

		
	
	#trick here. after building a wire, the edges change identity.
	#evidently, BRepBuilder_MakeWire makes copies of the underliying edges.
	h = hexagonlib.Hexagon(2.0,0 );
	wirelist = h.makeHexForBoundingBox((0,0,0), (100,100,0));
	w = wirelist[0];
	ee = Wrappers.Wire(w).edgesAsList();
	TestDisplay.display.showShape(w);
	#compute two intersections
	e1 = Wrappers.Edge(ee[5]);
	e2 = Wrappers.Edge(ee[55]);
	e1p = (e1.lastParameter - e1.firstParameter )/ 2;
	e2p = (e2.lastParameter - e2.firstParameter )/ 2;
	p1 = PointOnAnEdge(e1.edge,e1p ,e1.pointAtParameter(e1p));
	p2 = PointOnAnEdge(e2.edge,e2p ,e2.pointAtParameter(e2p));
	TestDisplay.display.showShape( Wrappers.make_vertex(p1.point));
	TestDisplay.display.showShape( Wrappers.make_vertex(p2.point));
	#cProfile.runctx('for i in range(1,20): ee=splitWire(w,[p2,p1])', globals(), locals(), filename="slicer.prof")
	#p = pstats.Stats('slicer.prof')
	#p.sort_stats('cum')
	#p.print_stats(.98);		

	t = Wrappers.Timer();
	ee = [];
	for i in range(1,1000):
		ee = splitWire(w,[p2,p1]);
		assert len(ee) == 1;
	
	print "Elapsed for 1000splits:",t.finishedString();
	#TestDisplay.display.showShape(ee);
	
def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
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
	#runProfiled('splitPerfTest()');
	splitPerfTest();
	print "Tests Complete.";
	
	TestDisplay.display.run();			
		