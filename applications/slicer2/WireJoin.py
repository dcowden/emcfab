"""
	Provides support to find the best way to connect a given point
	to a particular location on an edge
"""
import time

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



class WireJoiner:
	"""
		point is the point we'd like to join from
		wire is the wire we'd like to join to
	"""
	def __init__(self,wire,startPoint,trackWidth):
		self.wire = wire;
		self.startPoint = startPoint;
		self.trackWidth = trackWidth;
		
	#get the resulting wire
	@Util.printTiming
	def build(self):
		topoWire = Topo(self.wire);
		
		#compute closest point on the wire
		brp = BRepExtrema.BRepExtrema_DistShapeShape();
		brp.LoadS1(OCCUtil.make_vertex(self.startPoint));
		brp.LoadS2(self.wire);

		result = brp.Perform();    
		p1 = brp.PointOnShape2(1);
		wb = OCCUtil.WireBuilder();
		closestParam = None;
		if brp.SupportTypeShape2(1) == BRepExtrema.BRepExtrema_IsOnEdge:
			#closest point is a point along an edge
			interSectingEdge = OCCUtil.cast(brp.SupportOnShape2(1));
			closestParam = brp.ParOnEdgeS2(1);
		else:	
			#closest point is a point on a vertex, here we'll shorten one edge
			#in this case closest point is a vertex, so we dont have a param on an edge
			vertex = OCCUtil.cast(brp.SupportOnShape2(1));
			edges = [];     
			for e in  topoWire.edges_from_vertex(vertex):
			    edges.append(e);
			    
			    interSectingEdge = edges[0];
		    
		#compute parameter along one curve
		#break the edge into two new edges. account for a split distance between them.
		ej = EdgeJoin(interSectingEdge,self.startPoint,self.trackWidth ,closestParam); 
			
		#add lead-in edges
		for e in ej.connectingEdges:       
			wb.add(e);
		
		#now add all of the other edges in the wire except the original one that we split
		for e in topoWire.edges():
		    if not e.IsSame(interSectingEdge):
		        wb.add(e);
		
		for e in ej.otherEdges:
			wb.add(e);
			  
		return wb.wire();
	

""" 
	handles the logic of finding out the best way to 
	connect to a given edge from a particular point
	I THINK it is sound to simply use parameter math for distance along a curve, which is done here?
	
	It is a key assumption that the point we are joining from is already pretty close. No collision detection is done

	the basic heuristic here is to move about 2*distance along the curve
	so that the joint is at a shallow angle, then split the edge at that point.
	if that results in hitting a vertex, then we simply join to the vertex instead of in the middle
	of the edge.
	if distance is < trackWidth, then we are essentially already right on top of the edge, and thus we
	
	closestParam can be none-- if it is not supplied it is taken as the end of the edge closest to start point.
"""
class EdgeJoin():
	def __init__(self,edge, startPoint,trackWidth,closestParam=None):

		#jeeze i hate all these selfs.
		self.startPoint = startPoint;				
		(self.handleCurve, self.p1, self.p2  ) = brepTool.Curve(edge);
		self.curve = GeomAdaptor.GeomAdaptor_Curve(self.handleCurve);				
		if (self.p1>self.p2): 
			(self.p2,self.p1) = (self.p1,self.p2);
		self.edgeStart = self.curve.Value(self.p1);
		self.edgeEnd = self.curve.Value(self.p2);
		
		if closestParam == None:
			if self.startPoint.Distance(self.edgeStart) > self.startPoint.Distance(self.edgeEnd):
				self.closestParam = self.p2;
			else:
				self.closestParam = self.p1;
		else:
			self.closestParam = closestParam;

		assert self.closestParam >= self.p1 and closestParam <= self.p2;

		self.closestPoint = self.curve.Value(self.closestParam);		
		self.trackWidth = trackWidth;
		self.connectingPoint = None;		
		self.distanceFromStartPoint = startPoint.Distance(self.closestPoint);	
		self._compute();

		#EXPORTED (computed) values
		self.connectingPoint = None;
		self.connectingParam = None;
		self.connectingEdges = None; #edges to use at the beginning
		self.otherEdges = None; #left over edges, typically to add to the end of loop
	
		self._compute();
	

	#TODO there is a lot of duplicated code below, mostly just stuff swapped or signs swapped.
	#im sure it can be cleaned up
	def _compute(self):
		(p1,p,p2) = (self.p1,self.closestParam, self.p2); #p1 < p < p2
		
		#i think this works well for small distance. not as well if d is large.
	
		if (p - p1) < (2.0)*self.distanceFromStartPoint:
			#TODO: this code is nearly same as the branch below but with p1 and p2 swapped,
			#connect to start vertex
			self.connectingPoint = self.edgeStart;
			self.connectingParm = p1;
			
			#compute shortened edge
			e1 = OCCUtil.edgeFromTwoPointsOnCurve(self.handleCurve,p1+self.trackWidth,p2);
			self.otherEdges = [e1];
			
			#enter the wire by connecting start to vertex directly			
			self.connectingEdges = [ OCCUtil.edgeFromTwoPoints(self.startPoint,self.connectingPoint) ];
			
		elif (p2 - p )< (2.0)*self.distanceFromStartPoint:
			#connect to end vertex
			self.connectingPoint = self.edgeEnd;
			self.connectingParm = p2;
			
			#compute shortened edge
			e1 = OCCUtil.edgeFromTwoPointsOnCurve(self.handleCurve,p1,p2-self.trackWidth);
			self.otherEdges = [e1];
			
			#enter the wire by connecting start to vertex directly			
			self.connectingEdges = [ OCCUtil.edgeFromTwoPoints(self.startPoint,self.connectingPoint) ];			
		else:
			#connect to middle of the edge
			self.connectingEdges = [];
			firstEdge = None;
			#which end is closer?
			if (p - p1) > (p2 - p ): #p2 is closer 
				pTarget = p + (2.0)*self.distanceFromStartPoint;
				pEnd = pTarget - self.trackWidth;
				secondEdge = OCCUtil.edgeFromTwoPointsOnCurve(self.handleCurve,pTarget,p2);
				self.otherEdges = [OCCUtil.edgeFromTwoPointsOnCurve(self.handleCurve,p1,pEnd) ];
			else: #p1 is closer
				pTarget = p - (2.0)*self.distanceFromStartPoint;
				pEnd = pTarget + self.trackWidth;
				secondEdge = OCCUtil.edgeFromTwoPointsOnCurve(self.handleCurve,p1,pTarget);
				self.otherEdges = [OCCUtil.edgeFromTwoPointsOnCurve(self.handleCurve,pEnd,p2) ];
				
			self.connectingParam = pTarget;
			self.connectingPoint = self.curve.Value(pTarget);
			self.connectingEdges.append( OCCUtil.edgeFromTwoPoints(self.startPoint, self.connectingPoint));
			self.connectingEdges.append(secondEdge);

def TestWireJoiner():
	heartWire = TestObjects.makeHeartWire();
	
	#man this is hard to test without resorting to graphical test.
	#this is sort of a semi-automatic method
	testPoints = [];

	testPoints.append ( [ gp.gp_Pnt(-1.0,0,0),5]);
	testPoints.append ( [ gp.gp_Pnt(0,-1.0,0),5]);
	testPoints.append ( [ gp.gp_Pnt(-1.0,-1.0,0),5]);		
	testPoints.append ( [ gp.gp_Pnt(1.0,0,0 ),5]);
	testPoints.append ( [ gp.gp_Pnt(1.6,1.8,0),6]);
	testPoints.append ( [ gp.gp_Pnt(4.15,4.4,0),5] );
	testPoints.append ( [ gp.gp_Pnt(0.5,3.5,0),5] );	
	testPoints.append ( [ gp.gp_Pnt(4.0,4.7,0),6] );
		
	for tp in testPoints:
		display.EraseAll();
		display.DisplayColoredShape(heartWire,'GREEN');
		wj = WireJoiner(heartWire,tp[0],0.08);
		result = wj.build();
		assert len(Wrappers.Wire(result).edgesAsList()) == tp[1]
		display.DisplayColoredShape(result,'RED');
		time.sleep(1);
    
if __name__=='__main__':
	from OCC.Display.SimpleGui import *
	display, start_display, add_menu, add_function_to_menu = init_display()
	
	TestWireJoiner();
	
	start_display();	