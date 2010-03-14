"""
 Defines some useful wrappers for dealing with various constructs
"""
from OCC import BRep,gp,GeomAbs,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
#brt = BRepTools.BRepTools();
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();
#topexp = TopExp.TopExp()
#texp = TopExp.TopExp_Explorer();

import Utils

"""
	Point
"""
class Point:
	def __init__(self,point):
		self.point = point;
		
	def __str__(self):
		return "x=%0.3f,y=%0.3f,z=%0.3f" % (self.point.X(), self.point.Y(), self.point.Z());
		
"""
  Edge: provides useful functions for dealing with an edge shape
"""
class Edge:
	def __init__(self,edge):
		self.edge = edge;		
		hc = brepTool.Curve(edge);		
		self.curve = GeomAdaptor.GeomAdaptor_Curve(hc[0]); 
		self.firstParameter = hc[1];
		self.lastParameter = hc[2];		
		
		p1 = self.curve.Value(self.firstParameter);
		p2 = self.curve.Value(self.lastParameter);

		#compute the first and last points, which are very commonly used		
		if edge.Orientation() == TopAbs.TopAbs_FORWARD:
			self.reversed = True;
			self.firstPoint = p1;
			self.lastPoint = p2;
		else:
			self.reversed = False;
			self.firstPoint = p2;
			self.lastPoint = p1;
		
	def isLine(self):
		return self.curve.GetType() == GeomAbs.GeomAbs_Line;
	
	def isArc(self):
		return self.curve.GetType() == GeomAbs.GeomAbs_Circle;
	
	def discretePoints(self,deflection):
		"a generator for all the points on the edge, in forward order"
		gc = GCPnts.GCPnts_QuasiUniformDeflection(self.curve,deflection,self.firstParameter,self.lastParameter);
		
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
		
	def edges(self):
		"a generator for edges"
		bwe = BRepTools.BRepTools_WireExplorer(wire);
		while bwe.More():
			edge = bwe.Current();
			yield edge;
			bwe.Next();
		bwe.Clear();
		
if __name__=='__main__':
	print "Basic Wrappers"
	builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(gp.gp_Pnt(0,0,0),gp.gp_Pnt(1,1,1));
	builder.Build();
	edge = builder.Edge();
	hc = brepTool.Curve(edge);
	c  = GeomAdaptor.GeomAdaptor_Curve(hc[0]); 		
	p1 = c.Value(hc[1]);
	p2 = c.Value(hc[2]);
	#print str(edge1);