"""
	EdgeGraphs
	
	Utilties for representing wires and lists of edges in a graph.
	
"""

from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
import time,os,sys,string;
from OCC.Geom import *

from OCC.Utils.Topology import Topo
from OCC.Utils.Topology import WireExplorer
import itertools
import time
import networkx as nx

import Wrappers
import traverse
import OCCUtil
import Util
import hexagonlib

brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();

def tP(point,places=3):
	"""
	    return a tuple for a point.
		rounds to the number of places specified to ensure that
		rounding error does not result in non-coincident nodes
	"""
	return (round(point.X(),places), round(point.Y(),places) );
	

class PointOnAnEdge:
	"stores an intersection point"
	def __init__(self,edge,param,gppnt):
		self.edge = edge;
		self.param = param;
		self.hash = self.edge.__hash__();
		self.point = gppnt;
		self.node = tP(gppnt);
"""
	Graph of edges.
	Also provides finding a single edge in the chain,
	navigating along the chain in both directions,
	and navigating from the starting point of a particular edge.
	
	This is a bit different than a normal graph, it contains extra 
	functions to find edges using their native OCC edge object and parameter.
	
"""
class EdgeGraphBuilder:

	def __init__ (self):	
		"""
			dictionary contains each underlying edge, and all of 
			the nodes lying on that edge. multiple edge nodes
			can lie on a single edge because edge nodes might subdivide an edge
			
			Nodes in this graph are actually OCC edges.
			
			One major complexity comes from the fact that multiple nodes can share
			the same underlying edge-- this happens when a single edge is subdivided.
			the only difference is the parameters on the edge in that case.
			
		"""
		self.boundaryEdges = {}; #store boundary edges organized by hashed edge
		self.fillEdges = []; #ordered list of edges-- used when we walk the graph later. each 3-tuple (x, y, edgeObject )
		self.closeEdges = []; #list of boundary points that are too close to a fill contour
		self.graph = nx.Graph();
	"""
		adds an intersection point on the boundary 
		for later addition to the grid.
		
		After the entire 
	"""
	def addPointOnBoundaryEdge(self,edge,param,pnt):
		"add the provided point as an intersection"
		ec = self.boundaryEdges.setdefault(edge,[]); #ec will be empty list, or list of intersections for this edge
		ec.append(PointOnAnEdge(edge,param,pnt)); 

	"""
		adds an edge and both its endpoints
	"""
	def addSingleBoundaryEdge(self,edge):
		(f,l) = brepTool.Range(edge);
		(firstPoint, lastPoint) = OCCUtil.pointAtParameterList(edge, [f,l] );
		self.addPointOnBoundaryEdge(edge,f,firstPoint);
		self.addPointOnBoundaryEdge(edge,l,lastPoint);
	
	"""
		adds the edges of the wire
	"""
	def addBoundaryWire(self,wire):
		#for finding an edge node
		wr = Wrappers.Wire(wire);
		eS = wr.edgesAsSequence();

		for i in range(1,eS.Length()+1):
			e = OCCUtil.cast(eS.Value(i));
			self.addSingleBoundaryEdge(e);	

	"""
		Track pairs of edges that did not intersect, but were close to each other
		ctype is either EDGE or VERTEX-- the type of point that was too close to the edge
		boundaryPoint is a PointOnAnEdge object, the location on the boundary that was too close
	"""
	def addPointsTooClose(self,boundaryPoint,ctype):
		#
		self.closeEdges.append((boundaryPoint,ctype));

	"""
		Add a list of edges.
		If the list has only a single edge, it is simply added.
		if the list contains multiple edges, the end points are used
		to create the nodes of the list
		
		***
		TODO: i think we could maybe speed this up because for fillwires
		we already have the point and parameter from the intersection calcs--
		why not preserve them rather than re-compute?
		***
	"""
	def addFillEdges(self,edgeList):

		if len(edgeList) == 1:
			(f,l) = brepTool.Range(edgeList[0]);
			p1 = tP( OCCUtil.pointAtParameter( edgeList[0],f));
			p2 = tP( OCCUtil.pointAtParameter( edgeList[0],l));
		else:
			firstEdge = edgeList[0];
			lastEdge = edgeList[-1];
			(f,l) = brepTool.Range(firstEdge);
			(n,m) = brepTool.Range(lastEdge);
			
			if firstEdge.Orientation() == TopAbs.TopAbs_FORWARD:
				p1 = tP( OCCUtil.pointAtParameter( firstEdge,f));
			else:
				p1 = tP( OCCUtil.pointAtParameter ( firstEdge,l));
			
			if lastEdge.Orientation() == TopAbs.TopAbs_FORWARD:	
				p2 = tP(OCCUtil.pointAtParameter(lastEdge,m ));
			else:
				p2 = tP(OCCUtil.pointAtParameter(lastEdge,n ));
		
		self.fillEdges.append( (p1,p2,edgeList )); #would be nice if edge lists and edges looked the same

	"""
		build a graph of the nodes.
		
		the reason we dont add them as we go along is because it is much faster
		to add the nodes and build the edges needed, rather than actually
		adding them and then dividing them as we go along.
		
		in the case where two nodes are connected by both fill and boundaries, we do not 
		want to add boundary connections in that case.
	"""
	def buildGraph(self):
		g = self.graph;
		
		#add fill edge nodes first
		for e in self.fillEdges:
			g.add_edge(e[0], e[1], {"type":'FILL', "edgeList":e[2]});
					
		#add boundary nodes. 
		for (edge, pointList ) in self.boundaryEdges.iteritems():
			#sort the points by parameter
			sortedPoints = sorted(pointList,key = lambda p: p.param );
			
			for (poe1,poe2) in Util.pairwise(sortedPoints):
				#dont add if there is already an edge
				if not g.has_edge(poe1.node,poe2.node):
					#here we need to trim each edge to limit it to the desired parameters
					g.add_edge(poe1.node,poe2.node,{"type":"BOUND", "edge": OCCUtil.trimmedEdge(edge,poe1.param,poe2.param)});

	"""
		walk the edges in a sensible order
	"""
	def walkEdges(self):
		return traverse.travelAll(self.graph,self.fillEdges,isFillEdge );
				
def isFillEdge(graph,n1,n2):
	return graph[n1][n2]["type"] == 'FILL';
	
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
			edgeList.append ( [ OCCUtil.trimmedEdge(currentIntersection.edge, currentIntersection.param, nextIntersection.param ) ] ); 
		else:
			#intersections are not on the same edge.
			#add the fraction of the first edge
			(bp,ep) = brepTool.Range(currentIntersection.edge);
			edges = [];
			#print "adding piece of start edge."
			edges.append( OCCUtil.trimmedEdge(currentIntersection.edge,currentIntersection.param, ep));
	
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
				edge = OCCUtil.cast(topexp.Current() );
				edges.append(edge);
				#print "adding middle edge"
				topexp.Next();
	
			#add the last edge
			(bp,ep) = brepTool.Range(nextIntersection.edge);
			edges.append( OCCUtil.trimmedEdge(nextIntersection.edge,bp,nextIntersection.param));
			#print "adding last piece of edge"
			edgeList.append(edges);
	return edgeList;




def testSplitWire1():
	"""
		Test split wire function. there are two main cases:
		wires with intersection on different edges,
		and a wire with a single edge split in many places
	"""
		
	#case 1: a single edge with lots of intersections along its length
	e  = OCCUtil.edgeFromTwoPoints( gp.gp_Pnt(0,0,0),gp.gp_Pnt(5,0,0));
	w = OCCUtil.wireFromEdges([e]);
	
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

	assert length == 2.0;
	
def testSplitWire2():
	"intersections on different edges. one edge completely inside"
	
	e1 = OCCUtil.edgeFromTwoPoints(gp.gp_Pnt(0,0,0),gp.gp_Pnt(2,0,0));
	e2 = OCCUtil.edgeFromTwoPoints(gp.gp_Pnt(2,0,0),gp.gp_Pnt(5,0,0));
	e3 = OCCUtil.edgeFromTwoPoints(gp.gp_Pnt(5,0,0),gp.gp_Pnt(6,0,0));
	
	
	#trick here. after building a wire, the edges change identity.
	#evidently, BRepBuilder_MakeWire makes copies of the underliying edges.
	
	w = OCCUtil.wireFromEdges([e1,e2,e3]);
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
	#print "length=%0.3f" % length;
	assert length == 4.5;


		
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

	#compute two intersections
	e1 = Wrappers.Edge(ee[5]);
	e2 = Wrappers.Edge(ee[55]);
	e1p = (e1.lastParameter - e1.firstParameter )/ 2;
	e2p = (e2.lastParameter - e2.firstParameter )/ 2;
	p1 = PointOnAnEdge(e1.edge,e1p ,e1.pointAtParameter(e1p));
	p2 = PointOnAnEdge(e2.edge,e2p ,e2.pointAtParameter(e2p));

	t = Util.Timer();
	ee = [];
	for i in range(1,1000):
		ee = splitWire(w,[p2,p1]);
		assert len(ee) == 1;
	
	print "Elapsed for 1000splits:",t.finishedString();
	#TestDisplay.display.showShape(ee);
	
if __name__=='__main__':
	print "Basic Wrappers and Utilities Module"

	e1 = OCCUtil.edgeFromTwoPoints(gp.gp_Pnt(0,0,0),gp.gp_Pnt(2,0,0));
	
	"""
	t = OCCUtil.Timer();
	for i in range(1,20000):
		ew = OCCUtil.Edge(e1);
	print "Create 10000 edgewrappers= %0.3f" % t.elapsed();
	print ew.firstParameter, ew.lastParameter;
	
	t = OCCUtil.Timer();
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
	