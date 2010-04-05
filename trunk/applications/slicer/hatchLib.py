"""
	A Lightweight Hatching Library

"""
import os
import sys
import os.path
import wx
import logging
import time
import traceback

import math
from OCC  import gp
from OCC  import BRepAdaptor
from OCC import BRepBuilderAPI
from OCC import Extrema
from OCC import Approx
from OCC import GeomAbs
from OCC import BRepExtrema
from OCC import Geom
from OCC import BRepBndLib 
from OCC import Bnd;
from OCC import BRep;
from OCC import TopTools
from OCC import TopoDS
from OCC import TopAbs
import Wrappers
import TestDisplay



log = logging.getLogger('hatchLib');

ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();

"""
	convienient way to iterate in pairs with loop around to the start
	given a list, return it in pairs, with the last element wrapping around to the first
	if wrap is false, the last element is not returned.
	
	
	Example:
		lst = [1,2,3];
		
		ntuples(lst,2,True)
			(1,2)
			(2,3)
			(3,1)
		
		ntuples(lst,2,False)
			(1,2)
			(2,3)
		
		ntuples(lst,3,True)
			(1,2,3)
			(2,3,1)
			
		ntuples(lst,3,False)
			(1,2,3)
"""
def ntuples(lst, n,wrap=True):
	B = 0;
	if not wrap:
		B = 1;
	return zip(*[lst[i:]+lst[:(i-B)] for i in range(n)])

def pause():
	raw_input("Press Enter to continue");
	
#TODO: Vertex Nodes and Inodes need to be different classes extending Node
class Node:
	"represents a point on a boundary, with two references to the next and previous nodes"
	"the use of a wrapper class also allows the nodes to be moved slightly if needed, without"
	"affecting the ability to very quickly find the node using hash lookups"
	
	def __init__(self,point,isVertex):
	
		"these are used for all nodes"
		self.point = point;
		self.boundary = None; #the boundary this node lies upon		
		self.nextNode = None; #next node along the boundary, wrapping around
		self.prevNode = None; #previous node along the boundary, wrapping around
		self.vertex = isVertex; # a vertex is a node that is there because the curve changes along the wire
		self.used = False;

		
		#these only used for intersection ( not vertex ) nodes
		self.infillNode = None; #the other node that makes up the edge of this hatch line

		#for vertex nodes these values will be different, because the edges on eitherside
		#of the node are different. But for intersection nodes, the values will always match
		self.prevEdge = None;
		self.nextEdge = None;
		self.paramOnPrevEdge = None;
		self.paramOnNextEdge = None;
		
	def canMoveInfill(self):
		return self.infillNode and (not self.infillNode.used);
	
		
	@staticmethod
	def toString(node):
		if node.vertex:
			return "Vertex(%d), loc=[%0.2f, %0.2f, %0.2f]"  %  ( id(node), node.point.X(), node.point.Y(), node.point.Z() );
		else:
			return "INode(%d), Edge1= %d, Edge2=%d, P1=%0.2f,P2=%0.2f, loc=[%0.2f, %0.2f, %0.2f]"  %  ( id(node), hashE(node.prevEdge.edgeWrapper.edge), hashE(node.nextEdge.edgeWrapper.edge),node.paramOnPrevEdge,node.paramOnNextEdge, node.point.X(), node.point.Y(), node.point.Z() );

	def __str__(self):
		
		s =  Node.toString(self);
		if self.nextNode:
			s += "\n\t\tNext-->" + Node.toString(self.nextNode);
		if self.prevNode:
			s += "\n\t\tPrev-->" + Node.toString(self.prevNode);
		if self.infillNode:
			s += "\n\t\tinFill-->" + Node.toString(self.infillNode);
		return s;

def findNextInfillNode(node):
	"finds the next infill node in either direction"
	n = findNextInfillNodeForward(node.nextNode);
	if n:
		return n;
		
	n = findNextInfillNodeBackward(node.prevNode);
	if n:
		return n;
	
	return None;
	
def findNextInfillNodeForward(node,path=[]):
	"finds the next infill node in the forward direction, starting with supplied node"
	path = path +[node];
	if node.used:
		return None;
		
	if node.vertex:
		log.debug("node is a vertex, deferring");
		if node.nextNode:
			return  findNextInfillNodeForward(node.nextNode,path);
	else:
		if node.canMoveInfill():
			return path + [node];
			
	return None;		

def findNextInfillNodeBackward(node,path=[]):
	"finds the next infill node in the forward direction, starting with supplied node"
	if node.used:
		return None;
		
	if node.vertex:
		log.debug("node is a vertex, deferring");
		if node.nextNode:
			return  findNextInfillNodeBackward(node.prevNode,path);
	else:
		if node.canMoveInfill():
			return path + [node];
			
	return None;	

class BoundaryEdge:
	"represents a boundary edge. Composese a regular edge"
	def __init__ ( self,edge):
		self.edgeWrapper = Wrappers.Edge(edge);
		self.id = hashE(edge);
		self.startNode = None;
		self.endNode = None;
		self.middleNodes = [];
		
	def __str__(self):
		return "BoundaryEdge:\n  Start Node:%s \n  EndNode:%s" % ( str(self.startNode),str(self.endNode));
	
	def __repr__(self):
		return self.__str__();
		
def linkNodes(nodeList,wrapAround=False):
	"link all of the nodes in the list head-to-tail"
	
	for i in range(1,len(nodeList)):
		n1 = nodeList[i-1];
		n2 = nodeList[i];
		
		n1.nextNode = n2;
		n2.prevNode = n1;
		
	if wrapAround:
		n1 = nodeList[0];
		n2 = nodeList[len(nodeList)-1];
		n1.prevNode = n2;
		n2.nextNode = n1;
		

def hashE(edge):
	return edge.HashCode(1000000);

class SegmentedBoundary:
	"""
		represents a boundary.
		on construction the boundary will map its edges,
		
		The object can then provide wires that link nodes along its edges		
	"""

	def __init__(self,wire):

		self.wire = wire;
		self.tolerance = 0.000001;
		self.edgeHashMap = {};	 # a hash of boundary edge objects
		self.edges = [] #needed to store the lists in the original, wire order
		
		#self.edgeGraph = Wrappers.Graph();		
		
		#build a graph of the edges on the boundary
		#we'll use this to build edges between nodes later on
		log.debug("Building Edge Graph...");
		wr = Wrappers.Wire(wire);
		

		#store edges hashed by the OCC C++ object ID
		for e in wr.edges():
			be = BoundaryEdge(e);
			self.edges.append(be);
			h = hashE(e);
			print "Found Edge %d " % h;
			self.edgeHashMap[be.id] = be;

		
		#build nodes between the edges
		#traverse the edges forwards to create start nodes
		previousNode = None;
		previousEdge = None;
		
		for be in self.edges:
			log.debug("Procssing Edge %s" % be.id);
			#create a node at the start of the edge
			#this node is a vertex, it has a parameter on two edges
			node = Node(be.edgeWrapper.firstPoint,True);
			node.boundary = self;
			node.nextEdge = be;
			node.paramOnNextEdge = be.edgeWrapper.firstParameter;
			be.startNode = node;
			
			if previousNode:
				previousEdge.endNode = node;
				node.prevEdge = previousEdge;
				node.paramOnPrevEdge = previousEdge.edgeWrapper.lastParameter;
				
				#connect the nodes together
				previousNode.nextNode = node;
				node.prevNode = previousNode;
				
			#save current values for next pass through the loop
			previousNode = node;
			previousEdge = be;
		
		#now all are linked but the last node. of the last edge, which is 
		#handled differently depending on if the wire is closed or not
		firstNode = self.edges[0].startNode;
		lastEdge = self.edges[len(self.edges)-1];
		
		if wire.Closed():
			log.debug("Wire Is Closed, linking end to start");
			lastEdge.endNode = firstNode;
			lastEdge.startNode.nextNode = firstNode;
			firstNode.prevEdge = lastEdge;
			firstNode.paramOnPrevEdge = lastEdge.edgeWrapper.lastParameter;
			firstNode.prevNode = lastEdge.startNode;
			
		else:
			#last edge gets a new node
			node = Node(previousEdgeW.edgeWrapper.lastPoint,True);
			node.boundary = self;
			node.prevEdge = lastEdge;
			lastEdge.endNode = node;

			firstNode.prevNode = node;
			node.nextNode = firstNode;
			
						
		print self.edgeHashMap;
		#pause();
		
		
	def newIntersectionNode(self, edge,edgeParam, point):
		"returns a new intersection node on this boundary."
		"the boundary will adjust neighbors to work it into the boundary"
	
		be = self.edgeHashMap[hashE(edge)];
		#make a new node to insert into the list
		newNode = Node(point,False); #an intersection point
		
		newNode.boundary = self;
		
		#bit of a hack-- a node in the middle of an edge
		#gets simulated values for next and previous
		#edges, just like a vertex. that makes making edges easier later.

		newNode.paramOnPrevEdge = edgeParam;
		newNode.paramOnNextEdge = edgeParam;
		newNode.prevEdge = be;
		newNode.nextEdge = be;
			
		be.middleNodes.append(newNode);		
		be.middleNodes.sort(cmp= lambda x,y: cmp(x.paramOnPrevEdge,y.paramOnPrevEdge));
		
		if be.edgeWrapper.edge.Orientation() != TopAbs.TopAbs_FORWARD:
			be.middleNodes.reverse();
	
		#now link the nodes together
		tmpList = [];
		tmpList.append(be.startNode);
		tmpList.extend(be.middleNodes);
		tmpList.append(be.endNode);
		linkNodes(tmpList,False);
		
		return newNode;
		
	def edgesToInFillNode(self,startNode):
		"returns a list of edges and the node that should be used next"
		"if no next code can be found, None is returned"
		assert startNode.boundary==self,"Expected to get a startnode on my boundary";		
		
		#search the node graph. if a vertex is encountered, it means we'll have to add an edge
		pathToNextInfill = findNextInfillNode(startNode);
		
		log.debug("Path has %d Nodes In it" % len(pathToNextInfill) );
		
		if pathToNextInfill == None:
			log.warn("Could not find valid path to the next infill");
			return None;
			
		edgesToReturn = [];
		
		#ok we have a path, starting with startNode and ending
		#with a node that is an infill node
		edgesToReturn = []; #the edges that will become our return list
		
		for (startNode,endNode) in ntuples(pathToNextInfill,2,False):  #iterate over the list in pairs of nodes
			log.debug("Start Node:" + str(startNode));
			log.debug("End Node:" + str(endNode));
			
			assert startNode.nextEdge == endNode.prevEdge, "These nodes should be on either side of the same edge"
			
			currentEdge = startNode.nextEdge;
			edgesToReturn.append(currentEdge.edgeWrapper.trimmedEdge(startNode.paramOnNextEdge,endNode.paramOnPrevEdge) );

			startNode.used = True;
			endNode.used = True;
		
		log.info("Path Contains %d Edges." % len(edgesToReturn) );
		lastNode = pathToNextInfill.pop();
		return  [edgesToReturn,lastNode];
		
	"""
	def edgesBetweenNodes(self,startNode, endNode ):
		
			builds a wire between the two nodes.
			the wire will consist of one edge if the two nodes
			are on the same edge. but if they are on different edges,
			the wire will contain all of the segments inbetween
			all of the nodes used as a part of the wire ( vertex or not ) will be marked used so they are not used again.
			THIS ALGORITHM assumes that the edges in the wire are oriented correctly, head to tail
		

		assert endNode.boundary == self,"Expected to get an endNode on my boundary";
		
		if startNode.edge == endNode.edge:
			log.debug("Nodes are on the same edge. Returning a single trimmed edge");
			log.debug("Making Edge between parameters %0.2f and %0.2f on EdgeID %d" % (startNode.edgeParameter, endNode.edgeParameter, hashE(startNode.edge)) );
			ew = Wrappers.Edge(startNode.edge);
			return  [ew.trimmedEdge(startNode.edgeParameter,endNode.edgeParameter)];
		
		log.debug("nodes are on different edges. need to find the best path");

		edgesToReturn = []; #the edges that will become our wire
		log.debug("Finding Path between edges %d and %d " % (hashE(startNode.edge), hashE(endNode.edge) ));
		path = self.edgeGraph.find_shortest_path(hashE(startNode.edge), hashE(endNode.edge));		
		log.debug("Shortest Path Contains %d segments" % len(path ) );
		
		#the list includes the start and end edges. there are intermediate 
		#edges if the list is bigger than two
		print path;
		print self.edgeHashMap;
		#get the portion of the first edge
		#then determine if we are going along the wire or backwards
		ew1 = Wrappers.Edge(startNode.edge);
		ew2 = Wrappers.Edge(self.edgeHashMap[path[1]].edge );
		
		if ew1.lastPoint.Distance(ew2.firstPoint) < self.tolerance:
			log.debug("Edges are aligned with the wire");
			movingPositive= True;
			edgesToReturn.append( ew1.trimmedEdge(startNode.edgeParameter,ew1.lastParameter));
		else:
			log.debug("Edges are reversed from the wire");
			movingPositive=False;
			edgesToReturn.append( ew1.trimmedEdge(startNode.edgeParameter,ew1.firstParameter));
		
		#get intermediate edges
		for i in range(1,len(path)-1): #executes only for edges in between first and last
			edge = self.edgeHashMap(path[i]).edge;
			if not movingPositive:
				edge = Wrappers.cast( edge.Reversed());
			edgesToReturn.append(edge);
			
			#mark these nodes used. the other nodes are marked as used by the hatch algo
			[n1,n2] = self.nodesByEdge[hashE(edge)];
			n1.used = True;
			n2.used = True;
			
		#get the portion of the last edge
		ew1 = Wrappers.Edge(endNode.edge);
		
		if movingPositive:
			edgesToReturn.append( ew1.trimmedEdge(ew1.firstParameter, endNode.edgeParameter));
		else:
			edgesToReturn.append( ew1.trimmedEdge(ew1.lastParameter, endNode.edgeParameter));

		log.debug("Path Contains %d edges." % len(edgesToReturn) );
		return  edgesToReturn;
	"""

class Hatcher:
	"class that accepts a shape and produces a set of edges from it"
	"usage: Hatcher(...) then hatch() then edges()"
	def __init__(self,wireList,zLevel,infillSpacing,reverse,bounds):
	
		#self.boundaryWires = Wrappers.makeWiresFromOffsetShape(shape);
		self.boundaryWires = wireList;
		
		self.boundaries = [];
		for w in wireList:
			self.boundaries.append(SegmentedBoundary(w));
			
		self.bounds = bounds;
		self.zLevel = zLevel;
		self.HATCH_PADDING = 1; #TODO, should be based on unit of measure
		self.infillSpacing = infillSpacing
		self.reverse = reverse
		self.allIntersectionNodes = [];

		
	def  hatch(self):
		"take the a slice and compute hatches"
		log.info("Hatching...");
		hatchEdges = self._makeHatchLines();
		log.debug( "Created %d Hatch Edges" % len(hatchEdges));
		
		#list of all intersection points
		self.allIntersectionNodes  = [];
		
		boundariesFound = {};
		#intersect each line with each boundary
		continueHatching = True;
		for hatchLine in hatchEdges:
			if not continueHatching:
				break;
			else:
				log.debug( "Moving to next Hatch Line");
				
			log.debug("Hatching Boundary..");
			foundBoundary = False;

			interSections = [];	#list of intersections for just this single hatch line	
			
			for boundary in self.boundaries:
				brp = BRepExtrema.BRepExtrema_DistShapeShape();
				brp.LoadS1(boundary.wire);
				brp.LoadS2(hatchLine );
				
				if brp.Perform() and brp.Value() < 0.001:
					boundariesFound[boundary] = True;
					
					#TODO: if the intersections in practics are always ordered in ascending X, we can avoid
					#a lot of this code
					#TODO need to handle the somewhat unusual cases that the intersection is
					#on a vertex
					for k in range(1,brp.NbSolution()+1):
						#make the node
						if brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsOnEdge:
							newNode = boundary.newIntersectionNode(
								Wrappers.cast(brp.SupportOnShape1(k)),
								brp.ParOnEdgeS1(k),
								brp.PointOnShape1(k) );
						else:
							raise ValueError("I dont know how to handle vertex intersections");

						interSections.append(newNode);
					
				else:
					log.debug( "No Intersections Found");
					if foundBoundary:
						log.debug("No intersections found after finding the boundary. Quitting.");
						break;

			#sort by increasing X
			interSections.sort(cmp= lambda x,y: cmp(x.point.X(),y.point.X()));

			if len(interSections) == 0 and (len(boundariesFound) ==  len(self.boundaryWires)):
				continueHatching = False;
			
			if len(interSections) % 2 == 1:
				print "Detected Odd Number of intersection points. This is ignored for now."
				continue;
			
			log.debug("found %d intersection Nodes." % len(interSections));
			#build the node tree by connecting nodes on the same edge
			i = 0;
			while i< len(interSections):
				n1 = interSections[i];
				n2 = interSections[i+1];
				n2.infillNode = n1;
				n1.infillNode = n2;
				i += 2;
			
			#add to the global list
			self.allIntersectionNodes .extend(interSections);

		for n in self.allIntersectionNodes:
			print n;
			TestDisplay.display.showShape(Wrappers.make_vertex(n.point));
		log.info("Finished Hatching.");
		

	def edges(self):
		log.debug("Following Node Map...");				
		nodesLeft = Hatcher.findUnTracedNodes(self.allIntersectionNodes );
		numNodesLeft = len(nodesLeft);
		
		if numNodesLeft == 0:
			log.warn("No Intersection Nodes were found!");
			return;
			
		currentNode = nodesLeft[0];
		findingInfillEdge = True; #used to alternate between boundary and infill edges
		
		while ( numNodesLeft > 0):
		
			currentNode.used = True;
			numNodesLeft -= 1;
			#TestDisplay.display.showShape(Wrappers.make_vertex(currentNode.point));
			if findingInfillEdge:
				log.debug("looking for an infill edge..");
				#find an infill edge
				if currentNode.canMoveInfill():
					
					#return edge
					newNode = currentNode.infillNode
					yield Hatcher.linearEdgeBetweenNodes(currentNode,newNode);
					currentNode = newNode;
					
					findingInfillEdge = False;
				else:
					log.debug("current node does not have an infill path available, finding a new starting node ");
					#must find a new infill edge		
					currentNode = self.findFirstInfillNode();

			else:
				log.debug("looking for a boundary edge..");
				nn = currentNode.boundary.edgesToInFillNode(currentNode);
				if nn:
					log.debug("next node is available for a move, using that one");
					for e in nn[0]:
						yield e;
					currentNode = nn[1];
				else:
					log.debug("current node does not have any boundary nodes available, selecting new infill node");
					#need to select a new infill Edge
					currentNode = self.findFirstInfillNode();
						
				findingInfillEdge = True;

		#done-- no nodes left
		log.debug("No More Nodes Left.");
		
	def _makeHatchLines(self):
		"Makes a set of hatch lines that cover the specified shape"
		log.debug("Making Hatch Lines...");
		
		hatchEdges = [];
		#this algo is a simple one that simply does 45 degree hatching
		xMin = self.bounds[0] - ( self.HATCH_PADDING);
		yMin = self.bounds[1] - ( self.HATCH_PADDING);		 
		xMax = self.bounds[2] + (self.HATCH_PADDING);
		yMax = self.bounds[3] + (self.HATCH_PADDING) ;
		dX = xMax  - xMin;
		dY = yMax - yMin;
		maxP = 1.414 * max(dX,dY); # the diagonal of the square. this is our indexing parameter

		
		for p in Wrappers.frange6(0,maxP,self.infillSpacing):				
			#make an edge to intersect
			dl = p * 1.414 #length of a side
			if self.reverse:
				#travelling lower left to upper right
				p1 = gp.gp_Pnt(xMin ,yMin + dl,self.zLevel);
				p2 = gp.gp_Pnt(xMin + dl,yMin, self.zLevel);			
			else:
				#upper left to lower right
				p1 = gp.gp_Pnt(xMin,yMax - dl,self.zLevel);
				p2 = gp.gp_Pnt(xMin + dl,yMax,self.zLevel );

			edge = Wrappers.edgeFromTwoPoints(p1,p2);
			#TestDisplay.display.showShape(edge);
			if edge:
				hatchEdges.append(edge);			
	
		
		return hatchEdges;		
		
	
	@staticmethod
	def findUnTracedNodes(listOfNodes):
		"finds nodes in the list that have not been traced"
		"a graph is complete if all nodes which have infill connections have been traced"
		un = [];
		for n in listOfNodes:
			if not n.used:
				un.append(n);
		return un;

	def findFirstInfillNode(self):
		"find the first node that can be part of an infill edge"
		for n in self.allIntersectionNodes:
			if n.infillNode and (not n.used) :
				return n;
				
	@staticmethod
	def linearEdgeBetweenNodes(startNode,endNode):
		"make a linear edge from two points "
		builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(startNode.point,endNode.point);
		builder.Build();
		if builder.IsDone():
			return builder.Edge();
		else:
			return None;	


def checkWalkClosedWire(boundary,forward=True):
	"walk a wire, make sure its possible to get back to the start"
	
	MAX_HOPS = len(boundary.edges)*2;
	firstNode = boundary.edges[0].startNode;
	
	if forward:
		node = firstNode.nextNode;
	else:
		node = firstNode.prevNode;
	numHops = 0;
	
	while node != firstNode and numHops < MAX_HOPS:
		if forward:
			node = node.nextNode;
		else:
			node = node.prevNode;
		numHops += 1;
	log.debug("Traversed Wire in %d hops" % numHops );
	assert numHops < MAX_HOPS,"Too many hops to traverse the wire"

def checkNode(node,desc):
	"checks a node to ensure it looks right"
	assert node != None,desc + "Node is null"
	assert node.nextNode !=None,desc + ":No next node";
	assert node.prevNode !=None,desc +":No prev Node";
	assert node.nextNode != node, desc +":Next is self reference";
	assert node.prevNode != node, desc+":Prev is self reference";
	assert node.prevEdge != None, desc+":No prev edge";
	assert node.nextEdge != None, desc+":No next edge";

def checkBoundary(sb):
	"checks a boundary for correctness"
	print "Segmented boundary for square wire....",
	assert len(sb.edges) == 4, "There should be 4 edges"
	
	#make sure each edge has a start and end node, and each of those
	#has a next and previous node
	for edge  in sb.edges:
		checkNode(edge.startNode,"StartNode");
		checkNode(edge.endNode,"EndNode");
		
	checkWalkClosedWire(sb,True);
	checkWalkClosedWire(sb,False);			
	print "OK"	

if __name__=='__main__':

	###Logging Configuration
	logging.basicConfig(level=logging.DEBUG,
						format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
						stream=sys.stdout)	
	print "******************************"
	print " HatchLib Unit Tests...."
	print "******************************"
	w = TestDisplay.makeSquareWire();
	w2=TestDisplay.makeCircleWire();
	
	print "* Basic Boundary",
	sb = SegmentedBoundary(w);
	checkBoundary(sb);
	print "OK"
	
	print "* Basic Intersection",
	#add an intersection node in the middle of the first edge
	firstEdge = sb.edges[0];
	ew = firstEdge.edgeWrapper;
	p = abs(ew.firstParameter - ew.lastParameter )/2;
	newNode = sb.newIntersectionNode(  ew.edge,p ,ew.curve.Value(p) );
	
	assert newNode != None,"New Node should not be null"
	assert newNode.nextNode.vertex,"Next node should be vertex"
	assert newNode.prevNode.vertex,"Prev node should be vertex"
	checkBoundary(sb);


	#add another intersection on the same edge
	newNode2 = sb.newIntersectionNode(ew.edge,p+0.2,ew.curve.Value(p+0.2));
	checkBoundary(sb);
	print sb.edgeHashMap;
	print "OK"
	
	
	pause();
	#d.showShape(w);
	#TestDisplay.display.eraseAll();
	#hatch it
	h  = Hatcher([w2,w],0,2,False,[ 0,0,10,10] );
	h.hatch();
	#time.sleep(40);
	for e in h.edges():
		TestDisplay.display.showShape(e);
			
		time.sleep(1);
	TestDisplay.display.run();

	