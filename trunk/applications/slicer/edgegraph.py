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
		
		#next is edges reachable from the end of the edge
		self.next = {};
		
		#prev is the edges reachable from the beginning of the edge
		self.prev = {};
	
	def addNext(self,node):
		self.next[node] = node;
	
	def addPrev(self,node):
		self.prev[node] = node;
	
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
	Linked List of edges.
	Also provides finding a single edge in the chain,
	navigating along the chain in both directions,
	and navigating from the starting point of a particular edge
"""
class EdgeGraph:

	def __init__ (self):
		"make an empty edgegraph"
	
		"""
			dictionary contains each underlying edge, and all of 
			the nodes lying on that edge. multiple edge nodes
			can lie on a single edge because edge nodes might subdivide an edge
		"""
		self.edges = {};
	
	def  __repr__(self):
		return str(self.edges.values());
		
	def firstNode(self):
		"get the first node"
		return self.edges.values()[0][0];
		
	def addNode(self,edgeNode):
		e = edgeNode.edge;
		
		if not self.edges.has_key(e):
			self.edges[e] = [];		
		l = self.edges[e];
		
		#l is a list of all nodes on the same edge.
		#reminder, a 'none' is an edge, not a vertex.
		l.append(edgeNode);
	
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
		
		#re-configure start and ending parameters
		newNode = EdgeNode(edge,param,original.p2,original.type );
		original.p2 = param;
		
		#re-wire beginning and ending nodes
		newNode.next = original.next;
		original.next = {};
		original.addNext(newNode);
		newNode.addPrev(original);
		
		#add to global list
		self.addNode(newNode);
		
		return newNode;
		
	def findEdge(self,edge,param):
		"""
			Find the edge having the parameter of
			the contained value. returns none if not found.
			typically used to find an intersection value, 
		"""
		
		#TODO: it would be good to find a faster way to 
		#do this,ie, to hash by parameter or sort instead
		#of looping through all
		
		for n in self.edges[edge]:
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
			
			newnode = EdgeNode(edge,te.firstParameter,te.lastParameter,type);
			self.addNode(newnode);
			
			if last:
				newnode.addPrev(newnode);
				last.addNext(newnode);
				
			last = newnode;

			if i == 1:
				firstNode = newnode;
			if i == self.edgeSeq.Length():
				lastNode = newnode;
				
		
		#link last and first edge if the wire is closed
		if wire.Closed():
			lastNode.addNext(firstNode);
			firstNode.addPrev(lastNode);
			

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


	

if __name__=='__main__':
	print "Basic Wrappers and Utilities Module"
	w = TestDisplay.makeCircleWire();
	w = TestDisplay.makeSquareWire();
	w = TestDisplay.makeReversedWire();
	
	eg = EdgeGraph();
	eg.addWire(w,'BOUND');

	#the big test-- split one of the edges
	e = eg.firstNode().edge;
	newNode = eg.divideEdge(e,2.5);

	e2 = newNode.edge;
	newNode2 = eg.divideEdge(e2,0.2333 );
	
	for en in followForwardGenerator(eg):
		time.sleep(1);
		e = en.newEdge();
		TestDisplay.display.showShape( TestDisplay.makeEdgeIndicator(e) );
		TestDisplay.display.showShape(e );	


	TestDisplay.display.run();			
		