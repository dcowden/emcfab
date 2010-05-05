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
from OCC import TColStd

import Wrappers
import TestDisplay


def normalEdgeAtParameter(edge, param, length ):
	"compute an edge having length at the specified parameter on the supplied curve:"

	ew = Wrappers.Edge(edge);
	zDir = gp.gp().DZ();
	zVec = gp.gp_Vec(zDir);
	
	tangent = gp.gp_Vec();
	tanpoint = gp.gp_Pnt();
	
	curve = ew.curve;
	curve.D1(param,tanpoint,tangent );
	axis = gp.gp_Ax1(tanpoint, gp.gp_Dir(tangent.Crossed(zVec) ) );
	
	line = Geom.Geom_Line(axis );
	return  BRepBuilderAPI.BRepBuilderAPI_MakeEdge(line.GetHandle(),0, length).Edge();

def normalEdgesAlongEdge(edge, length , interval):
	"compute an edge having length at the specified parameter on the supplied curve:"

	edgeList = [];
	
	ew = Wrappers.Edge(edge);
	zDir = gp.gp().DZ();
	zVec = gp.gp_Vec(zDir);
	
	curve = ew.curve;
	pStart = ew.firstParameter;
	pEnd = ew.lastParameter;
	
	for p in Wrappers.frange6(pStart,pEnd,interval):
		tangent = gp.gp_Vec();
		tanpoint = gp.gp_Pnt();
		curve.D1(p,tanpoint,tangent );
		axis = gp.gp_Ax1(tanpoint, gp.gp_Dir(tangent.Crossed(zVec) ) );
	
		line = Geom.Geom_Line(axis );
		e = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(line.GetHandle(),0, length).Edge();	
		if e:
			edgeList.append(e);
			
	return edgeList;
	
if __name__=='__main__':

	circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(0,0,0),gp.gp().DZ()),1);
	
	#make a few edges
	e1 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle,5,6.2).Edge();
	TestDisplay.display.showShape(e1);

	for e in normalEdgesAlongEdge(e1,0.1,0.1 ):
		TestDisplay.display.showShape(e);
		
	
	e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(gp.gp_Pnt(0,0,0),gp.gp_Pnt(1,1,0)).Edge();
	TestDisplay.display.showShape(e2);

	for e in normalEdgesAlongEdge(e2,0.1,0.1 ):
		TestDisplay.display.showShape(e);	

	TestDisplay.display.run();
