"""
	EdgeGraphs
	
	Utilties for representing wires and lists of edges in a graph.
	
"""

from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
import time,os,sys,string,logging;
from OCC.Geom import *
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();
import TestDisplay
import itertools
import Wrappers
import time
import networkx

def hashE(edge):
	return edge.HashCode(1000000);
	
"an edge in a connected set of edges."
class EdgeNode:

	"an edge node from an edge wrapper"
	def __init__(self,edgeWrapper,type):
		self.ew = edgeWrapper;
		self.edge = wrapper.edge;
		self.p1 = edgeWrapper.firstParameter;
		self.p2 = edgeWrapper.lastParameter;
		self.type = type;
	
	"an edge node in a linked list."
	"NOTE: the identify of this object is its first parameter and its edge reference."
	"These should not be changed after creation."
	def __init__(self,edge,p1,p2,type):
		"edge is a ref to an edge, with the beginning and ending parameters"
		"for the underlying curve"
		self.edge = edge;
		self.ew = Wrappers.Edge(edge);
		self.p1 = p1;
		self.p2 = p2;
		self.type =type;
			
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
			since multiple edgeNodes can be associated with a single edge,
			the key is a tuple consisting of the edge hash the the start parameter on the edge
		"""
		return ( hashE(self.edge),self.p1,self.p2);

	def __repr__(self):
		return  "EdgeNode: %d ( %0.3f - %0.3f ), Next:%d, Prev:%d" % ( hashE(self.edge),self.p1, self.p2, len(self.next), len(self.prev ));

			
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

		#key=nodeHash: value=dict(key=nodehash,value=node )
		self.next = {}; #next node associative dict

		#key=nodeHash: value=dict(key=nodehash,value=node )
		self.prev = {}; #prev node associative dict
				
	def firstNode(self):
		"get the first node"
		#return self.edges.values()[0][0];
		
	def addNode(self,edgeNode):
		e = edgeNode.edge;
		
		if not self.edges.has_key(e):
			self.edges[e] = {};		
		l = self.edges[e];
		
		l[edgeNode] = edgeNode;

	def linkPrev(fromNode, toNode):
		if not self.prev.has_key(fromNode):
			self.prev[fromNode] = {};
		d = self.prev[fromNode];
		d[toNode] = toNode;
		
	def linkNext(fromNode, toNode):
		if not self.next.has_key(fromNode):
			self.next[fromNode] = {};
		d = self.next[fromNode];
		d[toNode] = toNode;
	
	def removeAllBefore(self,edgeNode):
		"removes all of the nodes before the supplied node."
		"typically used to trim a chain"
		#follow chain of nodes, removing previous nodes
		
		#remove from edge dictionary
	
	def divideEdge(self,edge,param):
		"""
			split the edge at the provided parameter,
			creating two new edge nodes
		"""
		original = self.findEdge(edge,param);
		
		if original == None:
			raise ValueError,"Could not find edge having parameter %0.3f" % (param);
			
		#param must be between the bounds on the original edge
		assert param >= original.p1 and param <= original.p2;
		
		#create two new nodes to replace the original one
		newNode1 = EdgeNode(edge,original.p1,param,orginal.type);
		newNode2 = EdgeNode(edge,param,original.p2, original.type );
		self.addNode(newNode1);
		self.addNode(newNode2);
		
		#connect the two nodes together
		self.linkPrev(newNode2, newNode1);
		self.linkNext(newNode1,newNode2 );
		
		#re-connect siblings. note that since
		self.prev[newNode1] = self.prev[original];
		self.next[newNode2] = self.next[original];
				
		#delete the orginal node
		self.deleteNode(original);
				
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
		
		for n in self.edges[edge].values():
			if param >= n.p1 and param <= n.p2:
				return n;
		
		return None;

	def addWire(self,wire,type):
		"add all the edges of a wire. They will be connected together."
		"type is the node type for each edge"
		
		#for finding an edge node
		wr = Wrappers.Wire(wire);
		self.edgeSeq = wr.edgesAsSequence();
		firstNode = None;
		lastNode = None;
		last = None;
		for i in range(1,self.edgeSeq.Length()+1):

			edge = Wrappers.cast(self.edgeSeq.Value(i));
			te = Wrappers.Edge(edge);			
			newnode = EdgeNode(te,type);
			self.addNode(newnode);
			
			if last:
				self.linkPrev( newnode,last);
				self.linkNext(last,newnode );
				
			last = newnode;

			if i == 1:
				firstNode = newnode;
			if i == self.edgeSeq.Length():
				lastNode = newnode;

		#link last and first edge if the wire is closed
		if wire.Closed():
			self.linkPrev( firstNode,lastNode);
			self.linkNext( lastNode,firstNode);			

def followForwardGenerator(graph):
	"simple generator, follows the first next in each node, till no more are available, or till start"
	startNode = graph.firstNode();
	yield startNode;	
	nD = startNode.next;
	while  len(nD) > 0:
		nextNode = nD.values()[0];
		if nextNode == startNode:
			return;
		yield nextNode;
		nD = nextNode.next;

	return;


def tP(point):
	"return a tuple for a point"
	return (point.X(), point.Y(), point.Z() );


if __name__=='__main__':
	print "Basic Wrappers and Utilities Module"
	w = TestDisplay.makeCircleWire();
	w = TestDisplay.makeSquareWire();
	#w = TestDisplay.makeReversedWire();
	
	wr = Wrappers.Wire(w);

	g = networkx.Graph();
	
	for e in wr.edges():
		ew = Wrappers.Edge(e);
		g.add_edge(tP(ew.firstPoint),tP(ew.lastPoint),{'ew':ew});
		TestDisplay.display.showShape(e);
	
	#print g.nodes();
	#print g.edges();
	print g.get_edge_data((5.0,0.0,0.0),(5.0,5.0,0.0));
	
	#eg = EdgeGraph();
	#eg.addWire(w,'BOUND');

	#the big test-- split one of the edges
	#e = eg.firstNode().edge;
	#newNode = eg.divideEdge(e,2.5);

	#e2 = newNode.edge;
	#newNode2 = eg.divideEdge(e2,0.2333 );
	"""
	for en in followForwardGenerator(eg):
		time.sleep(1);
		e = en.newEdge();
		TestDisplay.display.showShape( TestDisplay.makeEdgeIndicator(e) );
		TestDisplay.display.showShape(e );	
	"""

	TestDisplay.display.run();			
		