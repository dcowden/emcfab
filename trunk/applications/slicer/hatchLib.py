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
from OCC import TopTools
from OCC import TopoDS

import Wrappers
import TestDisplay



log = logging.getLogger('hatchLib');

ts = TopoDS.TopoDS();

class Node:
	"represents a point on a boundary, with two references to the next and previous nodes"
	"the use of a wrapper class also allows the nodes to be moved slightly if needed, without"
	"affecting the ability to very quickly find the node using hash lookups"
	
	def __init__(self,point):
		self.point = point;
		
		self.boundary = None;
		self.nextNode = None; #next node along the boundary, wrapping around
		self.prevNode = None; #previous node along the boundary, wrapping around
		self.infillNode = None; #the other node that makes up the edge of this hatch line
		self.parameter = -1;
		
		self.used = False;
	
	def canMoveNext(self):
		"can we travel on the boundary to a node that would allow infill?"
		return self.nextNode and (not self.nextNode.used) and self.nextNode.infillNode ;
	
	def canMovePrev(self):
		"can we travel on the boundary to a node that would allow infill?"
		return self.prevNode and (not self.prevNode.used) and self.prevNode.infillNode ;
	
	def canMoveInfill(self):
		return self.infillNode and (not self.infillNode.used);
	
	def canMoveOnBoundary(self):
		return self.canMoveNext() or self.canMovePrev();
		
	@staticmethod
	def toString(node):
		return "Node(%d),Parameter=%0.2f, loc=[%0.2f, %0.2f, %0.2f]"  %  ( id(node), node.parameter, node.point.X(), node.point.Y(), node.point.Z() );
		
	def __str__(self):
		
		s =  Node.toString(self);
		if self.nextNode:
			s += "\n\t\tNext-->" + Node.toString(self.nextNode);
		if self.prevNode:
			s += "\n\t\tPrev-->" + Node.toString(self.prevNode);
		if self.infillNode:
			s += "\n\t\tinFill-->" + Node.toString(self.infillNode);
		return s;
		
class WrapAroundIndex:
	"an index adapter that converts indices that are out of bounds"
	"to a valid call wrapped around to the beginning or end of the list"
	"so -2 means the sent from the right end, and len() + 2 means the second index"
	def __init__(self,length):
		self.length = length;
		self.maxIndex = length - 1;
		
	def index(self,idx):
		if idx < -self. length or idx > self.length *2:
			raise ValueError,"Cannot loop around more than once"
		if idx < 0:
			return self.length + idx;
		if idx > self.maxIndex:
			return idx - self.length;

		return idx;
	
class SegmentedBoundary:
	"a boundary with a number of nodes along the boundary."
	def __init__(self,wire,intersectionNodes):
		log.debug("SegementedBoundary : %d nodes." % len(intersectionNodes ) );
		
		self.wire = wire;
		
		self.tolerance = 0.000001;

		self.nodes = set();
		
		#make a parameterized approximation of the wire
		#TODO: why do i have to approximate the curve instead of just using the curve itself?
		#the trouble is that this means that i cannot retain any real circles or splines, and instead
		#have to turn everything into line segments!
		self.curve  = BRepAdaptor.BRepAdaptor_CompCurve (wire);
		hc = BRepAdaptor.BRepAdaptor_HCompCurve(self.curve);
		curveHandle = hc.GetHandle();

		#approximate the curve using a tolerance
		#do we need the below?
		approx = Approx.Approx_Curve3d(curveHandle,self.tolerance,GeomAbs.GeomAbs_C2,2000,12);
		if approx.IsDone() and  approx.HasResult():
			# have the result
			anApproximatedCurve=approx.Curve();
			log.debug( "Curve is parameterized between %0.5f and %0.5f " % (  anApproximatedCurve.GetObject().FirstParameter(), anApproximatedCurve.GetObject().LastParameter() ));
			#anApproximatedCurve.GetObject().SetPeriodic();
			#log.debug( "Curve Periodic ="  + str(anApproximatedCurve.GetObject().IsPeriodic() ));
			self.approxCurve  = anApproximatedCurve;
		else:
			log.warn( "Failed to create approximation curve." );
		
		#project each point onto the curve and save the parameter
		for node in intersectionNodes:
			n =  self._findPointOnCurve(node.point);
			if not n:
				#TODO: weird problem where sometimes pointOnCurve does not match
				#I'm sure it is due to curve approximation
				log.warn("Could Not project a point on the curve. Lets act like this is not on the boundary.");
				continue;
			[ppt,param] = n;
			
			#replace the original point with the projected one.
			#this way the caller's reference to the provided node still works, but
			#we have adjusted the point underneath to match the projection
			node.point = ppt;
			node.boundary = self;
			node.parameter = param;
			self.nodes.add(node);

		sortedNodes= self._parameterSortedNodes();	
		#TODO this woorked, but connecting edges across the seams
		#of a parameterized curve didnt work
		#wrapIndex = WrapAroundIndex(len(sortedNodes));
		#for i in range(0,len(sortedNodes)):
		#	#these are safe-- the boundaries are 'wrapped around'
		#	prevNode = sortedNodes[wrapIndex.index(i-1)];
		#	thisNode = sortedNodes[wrapIndex.index(i)];
		#	nextNode = sortedNodes[wrapIndex.index(i+1)];
			
		#	thisNode.nextNode = nextNode;
		#	thisNode.prevNode = prevNode;

		for i in range(0,len(sortedNodes)):
			#these are safe-- the boundaries are 'wrapped around'
			if (i-1) >= 0:
				prevNode = sortedNodes[(i-1)];
			else:
				prevNode = None;
			
			if ( i+1) < (len(sortedNodes)  ):
				nextNode = sortedNodes[(i+1)];
			else:
				nextNode = None;
				
			thisNode = sortedNodes[i];
			thisNode.nextNode = nextNode;
			thisNode.prevNode = prevNode;
		

	@staticmethod
	def edgeBetweenNodes(startNode, endNode):
		"makes an edge between the specified two nodes, which must lie on the wire"
		if startNode.boundary  != endNode.boundary:
			raise ValueError,"Start Node and End Node are not on the same boundary"
			
		log.debug("Building Edge from parameter %0.2f to %0.2f" % ( startNode.parameter, endNode.parameter) );	
		builder =BRepBuilderAPI.BRepBuilderAPI_MakeEdge(startNode.boundary.approxCurve, startNode.parameter,endNode.parameter);
		builder.Build();
		if builder.IsDone():
			return builder.Edge();
		else:
			log.error( "Error building Edge: Error Number %d" % builder.Error());
			return None;
		
		
	def _findPointOnCurve(self,point):
		"finds the point and parameter on the wire"
		#or BRepExtrema_DistShapeShape ??
		#print "projecting point" + str(Wrappers.Point(point));
		
		projector = Extrema.Extrema_ExtPC(point,self.curve,self.tolerance);
		projector.Perform(point);
		if projector.IsDone():
			for i in range(1,projector.NbExt()+1):
				if projector.IsMin(i) and projector.Value(i) < self.tolerance:
					#print projector.Value(i),projector.Point(i).Value().X(), projector.Point(i).Value().Y(), projector.Point(i).Parameter()
					return [projector.Point(i).Value(), projector.Point(i).Parameter() ];
			log.warn("Could Not Find Close Enough Value!");
		else:
			log.warn("Could not project point.");
			return None;
	
	def _parameterSortedNodes(self):
		return sorted(self.nodes, cmp=lambda x,y: cmp(x.parameter, y.parameter));
	
	def __str__(self):
		"render this boundary"
		s = "Segmented Boundary, %d Nodes.\n" % len(self.nodes);
		for n in self._parameterSortedNodes():
			s+= ( "\t %s \n" % n);
		return s;

class Hatcher:
	"class that accepts a shape and produces a set of edges from it"
	"usage: Hatcher(...) then hatch() then edges()"
	def __init__(self,wireList,zLevel,infillSpacing,reverse,bounds):
	
		#self.boundaryWires = Wrappers.makeWiresFromOffsetShape(shape);
		self.boundaryWires = wireList;
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
		
		#list of the boundaries
		boundaryIntersections = {};
		segmentedBoundaries = [];
		
		for boundary in self.boundaryWires:
			#TestDisplay.display.showShape(boundary);
			boundaryIntersections[boundary] = [];
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
			
			#Geom2dAPI_InterCurveCurve is the 2-d equivalent, which would be much faster
			
			for boundary in self.boundaryWires:
				brp = BRepExtrema.BRepExtrema_DistShapeShape();
				brp.LoadS1(boundary);
				brp.LoadS2(hatchLine );
				
				if brp.Perform() and brp.Value() < 0.001:
					boundariesFound[boundary] = True;
					
					#TODO: if the intersections in practics are always ordered in ascending X, we can avoid
					#a lot of this code
					for k in range(1,brp.NbSolution()+1):
						#make the node
						newNode = Node(brp.PointOnShape1(k));						
						interSections.append(newNode);
						boundaryIntersections[boundary].append(newNode);
					
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

		#build all the intersection nodes for the boundaries
		for [k,v] in boundaryIntersections.iteritems():
			segmentedBoundaries.append(SegmentedBoundary(k,v));
		log.info("Found %d intersection Nodes" % len(self.allIntersectionNodes ));
		log.info("Finished Hatching.");
		
		#for n in self.allIntersectionNodes:
		#	print (str(n));

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
				
				if not currentNode.canMoveOnBoundary():
					log.debug("current node does not have any boundary nodes available, selecting new infill node");
					#need to select a new infill Edge
					currentNode = self.findFirstInfillNode();

				else:
					#can move either way, choose the next one
					#TODO: different algo needed here?
					if currentNode.canMoveNext():
						log.debug("next node is available for a move, using that one");
						yield SegmentedBoundary.edgeBetweenNodes(currentNode,currentNode.nextNode);
						currentNode = currentNode.nextNode;
					else:
						log.debug("prev node is available for a move, using that one");
						#TestDisplay.display.showShape(Wrappers.make_vertex(currentNode.prevNode.point));
						#print str(currentNode);
						yield SegmentedBoundary.edgeBetweenNodes(currentNode,currentNode.prevNode);
						currentNode = currentNode.prevNode;
						
				findingInfillEdge = True;
					
		
			#nodesLeft = Hatcher.findUnTracedNodes(self.allIntersectionNodes );

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
		
		
		"""
		    The below algo could be modified, but what is the point?
			most everyone will want 45 degrees, and that's ea
			
			
		#compute direction of line
		lineDir = gp.gp_Dir( 1,math.cos(math.radians(self.infillAngle)),self.zLevel );
		
		#disable built-in setting
		angleRads = math.radians(45.0);
		xSpacing = self.infillSpacing / math.sin(angleRads);		
		xStart = xMin - ( xMax  - xMin );
		xStop = xMax;
		#tan theta = op/adj , adj =op / tan 

		
		if self.infillAngle == 90:
			raise Exception, "90 Degree Hatching is not supported"
			
		if self.infillAngle < 90:
			xStart = ( xMin - (yMax - yMin)/math.tan(angleRads));
			xStop = xMax;			
			log.warn("xspacing = %0.2f, start = %0.2f, stop= %0.2f" %( xSpacing ,xStart, xStop) );
			for xN in Wrappers.frange6(xStart,xStop,xSpacing):
				
				#make an edge to intersect
				p1 = gp.gp_Pnt(xN,yMin,self.zLevel);
				p2 = gp.gp_Pnt(xMax, (xMax - xN)* math.tan(angleRads) + yMin, self.zLevel);
			
				edge = Wrappers.edgeFromTwoPoints(p1,p2);
				#TestDisplay.display.showShape(edge);
				hatchEdges.append(edge);			
		else:
			xStart = xMin;
			xStop = ( xMin - (yMax - yMin)/math.tan(angleRads));
			log.warn("xspacing = %0.2f, start = %0.2f, stop= %0.2f" %( xSpacing ,xStart, xStop) );
			for xN in Wrappers.frange6(xStart,xStop,xSpacing):
				
				#make an edge to intersect
				p1 = gp.gp_Pnt(xMin, ( xN -xMax )* math.tan(angleRads) + yMin, self.zLevel);
				p2 = gp.gp_Pnt(xN,yMin,self.zLevel);
				
			
				edge = Wrappers.edgeFromTwoPoints(p1,p2);
				TestDisplay.display.showShape(edge);
				hatchEdges.append(edge);
		"""
		
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


if __name__=='__main__':

	###Logging Configuration
	logging.basicConfig(level=logging.DEBUG,
						format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
						stream=sys.stdout)	
	w = TestDisplay.makeSquareWire();
	w2=TestDisplay.makeCircleWire();
	#TestDisplay.display.showShape(w2);
	#TestDisplay.display.showShape(w);
	
	#d.showShape(w);
	#TestDisplay.display.eraseAll();
	#hatch it
	h  = Hatcher([w2,w],0,0.1,False,[ 0,0,10,10] );
	h.hatch();
	#time.sleep(40);
	for e in h.edges():
		if e:
			TestDisplay.display.showShape(e);
		#time.sleep(.5);
	TestDisplay.display.run();

	