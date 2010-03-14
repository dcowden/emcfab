"""	
	Utility Functions

"""
import os
import sys
from OCC import TopoDS, TopExp, TopAbs,BRep,gp,GeomAbs,GeomAPI,BRepBuilderAPI

def edgeFromTwoPoints(p1,p2):
	builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2);
	builder.Build();
	if builder.IsDone():
		return builder.Edge();
	else:
		return None;
