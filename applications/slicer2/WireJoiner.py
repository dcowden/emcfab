"""
	Provides support to find the best way to connect a given point
	to a particular location on an edge
"""
import time,math

from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp,Approx,BRepLib
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
from OCC import BRepExtrema,TColgp
from OCC import ShapeAnalysis
from OCC import ShapeFix,ShapeExtend
from OCC.Utils import Topo


import OCCUtil
import TestObjects
import Wrappers
import Util

brt = BRepTools.BRepTools();
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();

MAX_DISTANCE=99999.0;
"""
	Class that will compute the best way to join a point
	to another wire. The assumption is that a joining wire will be 
	created between the startPoint and the selected vertex.
	
	The algorithm is pretty complex, the considerations are:
	
	* we want to prefer joining to vertices that are close	
	* minimize angle between the joint segment and the original edge
	* minimize angle between joint segment and the first edge we'll draw on the target wire
	
	Usage: construct object,
	  call build()
	  then select return values as needed
	  
"""

class WireJoiner:
	"""
		point is the point we'd like to join from
		wire is the wire we'd like to join to
	"""
	def __init__(self,jointRequest,tolerance=None, ):
		self.request = jointRequest;
		
		if tolerance is None:
			self.tolerance = (3.5)*self.request.trackWidth;
		else:
			self.tolerance = tolerance;
	#@Util.printTiming
	"""
		a lot of the work is done inside of JointSolution and JoinAtVertex,
		this class simply inspects all of the possible solutions and chooses the best one, when there are multiple
		choices
	"""
	@Util.printTiming
	def build(self):
	
		wire = self.request.wire;
		topoWire = Topo(wire);
		#get all the vertices for the wire
		#these are sorted by ascending distance
		choices = OCCUtil.nearestVertices([wire],self.request.startPoint,MAX_DISTANCE);
		#print "There are %d choices" % len(choices);
		bestSolution = None;
		for (wire,vertex,point,distance) in choices:
			#print "vertex: distance %0.3f" % distance;
			sol =  JoinAtVertex(self.request,vertex).compute();
			if distance < self.tolerance:
				#compute a joint solution based on this vertex
				if bestSolution is None:
					bestSolution = sol;				
				elif sol.isBetterThan(bestSolution):
					#print "switching solution to another vertex!"
					bestSolution = sol;						
			else:
				#all of the points below this one will be even less attractive, and
				#entry angle doesnt matter because we'll probably rapid there. Thus,
				#simply choose the next one
				if bestSolution:
					break;
				else:
					sol.isJoint = False;
					bestSolution = sol;
		
		#at this point we have the best solution, which is either a joint to a vertex
		#with the best combination of angles, or we have a move to then nearest vertex
		bestSolution.buildWire();
		return bestSolution;

"""
	a starting point for joining to a wire
"""
class JointRequest:
	def __init__(self,point,approachVector,trackWidth,wire):
		self.startPoint = point;
		self.startVector = approachVector;
		self.trackWidth = trackWidth;
		self.wire = wire;
		

class JointSolution:

	def __init__(self,jointRequest,pointOnEdge):
		
		#items we compute as soon as we know
		#what edge we are on.
		self.jointRequest = jointRequest;		
		
		#computed values
		self.pointOnEdge = pointOnEdge; #stores information about paramete,point, and vertex
		self.entryPoint = self.pointOnEdge.point;
		self.isJoint = True; #indicates if the solutoin allows direct joining or requires a long move
		self.exitVertex = None; # the last vertex on this wire after it is trimmed
		self.approachAngle = None; #the angle between the first edge and the startVector
		self.jointSegmentAngle = None; #the anble between the first edge and the line joining the start point to the start vertex
		self.trimDistance = None; #the amount that the last edge should be trimmed, based on the angle between the edges
		
		#compute angles based on the start points
		self.approachAngle = jointRequest.startVector.Angle(pointOnEdge.vector );
		
		vec = gp.gp_Vec(jointRequest.startPoint,pointOnEdge.point);
		
		#NOTE: since it is possible to travel an edge backwards, 
		#take the minimum angle after trying it reversed
		#also, if the joint angles are close, we defer to the approachAngle to avoid switchbacks when possible.
		self.jointSegmentAngle = min( vec.Angle( pointOnEdge.vector), vec.Reversed().Angle(pointOnEdge.vector));
		
		#computed when we ask for a wire computation.
		self.wire = None;
		self.trimmedEdge = None;
		self.trimmedPoint = None;
		self.trimmedVec = None;
	"""
		compute the wire corresponding to the solution.
	"""
	def buildWire(self):
		
		#simply do the work of the trimming and construction.
		wB = OCCUtil.WireBuilder();
		wire = self.jointRequest.wire;
		tw = Topo(wire);
		edgeToTrim = self.pointOnEdge.edge;
		
		#shorten the selected edge
		#TODO: maybe simplify this to return pointOnEdge?
		(self.trimmedEdge,self.trimmedPoint,self.trimmedVec) = OCCUtil.shortenEdge(edgeToTrim,self.pointOnEdge.point ,self.trimDistance);
			
		wB.add(self.trimmedEdge);
		#add the edges we didnt touch
		for e in tw.edges():
			if not e.IsSame(edgeToTrim):
				wB.add(e);
		self.wire = wB.wire(); 


	def isAngleSharp(self,angle):
		#is an angle sharp?  an angle of pi or any multiple is sharp
		#assume that we wont receive any angles that are less than 360degrees
		return abs(math.pi - angle) <  ( math.pi / 8.0 )
	

	"Is this solution preferable to another one?"
	#TODO: optimize performance by pre-computing.
	#NOTE: the best solution matches the edge direction best.
	#that means that a bigger angle is a less desirable match. 
	#but, since we trim the edge that is selected, that logic is reversed,
	#IE, a 'better' edge is the one that is better to trim, not to follow!
	#also, keep in mind that it is permissible to travel an edge backwards,
	#so we have to try each edge both directions
	
	#if the two joint angles are within about 15 degrees, it is best to choose based on the approach angle instead
	def isBetterThan(self,other):
		if abs(self.jointSegmentAngle - other.jointSegmentAngle) < ( math.pi / 180.0 * 25.0 ):
			return ( self.jointSegmentAngle > other.jointSegmentAngle);
		else:
			#print "comparing approahc angles..."
			return ( self.approachAngle > other.approachAngle );

	def hasSharpAngles(self):
		return self.isAngleSharp(self.jointSegmentAngle) or self.isAngleSharp(self.approachAngle)
	
"""
	given a single vertex on a wire, and an approach vector,
	compute the best way to trim/join at this point.
	
	This is based on the approach vector, the start vector, etc.

	The goal of the separate class is to allow inspecting the 
	quality of the solution separately from computing it, since
	it is sometimes necessary to inspect the soution at several vertices before choosing the best
	one.

	compute() returns a solution that will yield a trimmed wire, but does not
	do the actual trimming and computations until the solutoin's wire() method is called.
	this allows the various solutions to be computed and compared before the cost of constructing occurs.
	
"""
class JoinAtVertex:
	def __init__(self,jointRequest, vertex):
		self.request = jointRequest;
		self.vertex = vertex;
		
		#these are references for convienience
		self.solution = None;		

	"""
		compute the best solution.
		This method does NOT compute the trimmed wire,
		that's not done until we actually know we need to do it.
		return None, see other methods to inspect the solution
	"""
	def compute(self):
		tw = Topo(self.request.wire);
		eL = list(tw.edges_from_vertex(self.vertex));
	  
		if len(eL) > 2:  raise Exception("Cannot trim wire at vertex with more than two edges");	
		if len(eL) == 0:  raise Exception("Vertex has no edges, cannot trim")
		
		allVertices = list(tw.vertices());
			
		#only one edge accessible from this vertex
		if len(eL) == 1:
			edgeWrapper = Wrappers.Edge(eL[0]);
			pointObj = edgeWrapper.getPointAtVertex(self.vertex);
			self.solution = JointSolution(self.request,pointObj);
			self.solution.trimDistance = self.request.trackWidth; #for a single edge, simply trim the path width
	   		return self.solution;
	   	
		#else len(eL) == 2. this is the most common case.
		(edge1,edge2) = eL[:2];		
		edgeWrapper1 = Wrappers.Edge(eL[0]);
		edgeWrapper2 = Wrappers.Edge(eL[1]);

		#compute solutions using both edges
		pointOnEdge1 = edgeWrapper1.getPointAtVertex(self.vertex);
		pointOnEdge2 = edgeWrapper2.getPointAtVertex(self.vertex);
		solution1 = JointSolution(self.request,pointOnEdge1);
		solution2 = JointSolution(self.request,pointOnEdge2);
		
		#compute the trim distance, which is based on the angles between the edges
		#edges with vectors pi apart overlap, and 0 degrees are aligned
		#TODO: this assumes all angles are between zero and 2pi
		
		angleAtVertex = pointOnEdge1.vector.Angle(pointOnEdge2.vector);
		angleDiff = abs(math.pi - angleAtVertex)
		
		trimWidth = self.request.trackWidth;
		
		if angleDiff < 0.1:
			#print "Trimmed Wire: two edges are very close to each other!"
			#TODO: here, if one of the edges is a curve we can re-evaluate along the curve
			#to find a better location. If the cuves are lines, the two should be removed.
			#for now, though, we'll handle this in the shortening routine, which will
			#essentially never allow replacing an entire edge
			trimDistance = trimWidth*5.0;
		elif  angleDiff < (math.pi / 2):
			#print "Trimmed Wire based on sin rule"
			trimDistance =  trimWidth / 2 * (1 + (1 / math.sin(angleAtVertex)));
		else:
			#print "No Trim was necessary"
			trimDistance = trimWidth /2 ;

		solution1.trimDistance = trimDistance;
		solution2.trimDistance = trimDistance;	
		
		#choose the solution that is best. ideally, we want wide angles,
		#remember, these are direction vectors, so small angles means 'same direction' which is good,
		#while pi ( 180 ) angles are bad, becaues the paths switch back onto themselves.
		if solution1.isBetterThan(solution2):
			self.solution = solution1;
		else:
			self.solution = solution2;
		
		if self.solution is None:
			raise Exception("No solution selected, this should not ever occur.");
	
		return self.solution;

def TestWireJoiner():
	wire = TestObjects.makeOffsetTestWire();
	conditions= [];
	
	
	#each test condition is two points, representing an edge
	#that was drawn last.
	#in these tests, the red curve is the trimmed one, and the green is the original.
	#the yellow markers on the leadin edges and the joining segment is closer to the end
	
	#in each case, the solution found should minimize overlap and trim the other edge
	#as necessary.
	
	conditions.append((gp.gp_Pnt(11.2,1.0,0),gp.gp_Pnt(11.2,0.0,0)) );
	conditions.append((gp.gp_Pnt(11.2,-1.0,0),gp.gp_Pnt(11.2,0.0,0)) );
	conditions.append((gp.gp_Pnt(15.2,0.0,0),gp.gp_Pnt(11.2,0.0,0)) );
	conditions.append((gp.gp_Pnt(11.2,-3.0,0),gp.gp_Pnt(11.2,-1.0,0)) );
	conditions.append((gp.gp_Pnt(20.2,19.0,0),gp.gp_Pnt(20.2,20.0,0)) );
	conditions.append((gp.gp_Pnt(30.2,25.0,0),gp.gp_Pnt(20.2,20.0,0)) );
	
	for (startPoint,endPoint) in conditions:
		display.EraseAll();
		display.DisplayColoredShape(wire, 'GREEN');
		initVector = OCCUtil.edgeFromTwoPoints(startPoint,endPoint);
		display.DisplayColoredShape(OCCUtil.edgeFromTwoPoints(startPoint,endPoint),'BLUE');
		display.DisplayColoredShape(TestObjects.makeEdgeIndicator(initVector));	   
		vec = gp.gp_Vec(startPoint,endPoint);
		jrequest = JointRequest( endPoint,vec,0.8,wire);		
		wj = WireJoiner(jrequest,4.0); #this is a huge tolerance, but helps for testing
		solution = wj.build();
		display.DisplayColoredShape(solution.wire,'RED');
		entryEdge = OCCUtil.edgeFromTwoPoints(endPoint,solution.entryPoint);
		display.DisplayColoredShape(TestObjects.makeEdgeIndicator(entryEdge));
		if solution.isJoint:
			display.DisplayColoredShape(entryEdge,'WHITE');			
		else:
			display.DisplayColoredShape(entryEdge,'YELLOW');
		time.sleep(5);
	    
if __name__=='__main__':
	from OCC.Display.SimpleGui import *
	display, start_display, add_menu, add_function_to_menu = init_display()
	
	TestWireJoiner();
	
	start_display();	