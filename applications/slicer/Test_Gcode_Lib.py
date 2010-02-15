"""
	Test Cases for Gcode_Lib
	
	The library is currently a 3-axis library only.
	It is assumed that the axes of the model correspond to those of the machine.
	
	Copyright [2010] [Dave Cowden ( dave.cowden@gmail.com)]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0	

"""

from OCC.gp import *
from OCC.BRepBuilderAPI import *
from OCC.GeomAPI import *
from OCC.Geom import *
from OCC.GeomAbs import *
from OCC.TColgp import *
from OCC.Geom2d import *
from OCC.gp import *
from OCC.BRepBuilderAPI import *
from OCC.TColgp import *
from OCC.BRepOffsetAPI import *
from OCC.GeomAbs import *
from OCC.BRepPrimAPI import *
from OCC.Utils.Topology import Topo
from OCC.BRep import *
from OCC.Precision import *
from OCC.BRepLib import *
import Gcode_Lib
import math

def squareWaveWire():
	#make a squarewave wire
	#note the naive move at the end, which should be a single move to 5,5 
	#(if the feedrate is the same )
	Z=0;
	pts = [[0,0,Z],[1,0,Z],[1,1,Z],[2,1,Z],[3,3,Z],[4,4,Z],[5,5,Z]];
	return wireFrom2DPoints(pts);

def pointFromArray(apt ):
	return gp_Pnt(apt[0],apt[1],apt[2]);
	
def wireFrom2DPoints(points):
	MW = BRepBuilderAPI_MakeWire();
	lastPoint = points[0];
	first=True;
	for pt in points:
		if first:
			first=False;
			continue;
		edge = BRepBuilderAPI_MakeEdge(pointFromArray(lastPoint),pointFromArray(pt)).Edge();
		MW.Add(edge);
		lastPoint = pt;
	return MW.Wire();
	
def makeSimpleTestWire():
    circle2 = gp_Circ(gp_Ax2(gp_Pnt(40,40,2),gp_Dir(0,0,1)),10)
    Edge4 = BRepBuilderAPI_MakeEdge(circle2,gp_Pnt(40,50,2),gp_Pnt(50,40,2)).Edge()
    ExistingWire2 = BRepBuilderAPI_MakeWire(Edge4).Wire()
    P1 = gp_Pnt(50,40,2)
    P2 = gp_Pnt(80,40,2) #5,204,0
    Edge5 = BRepBuilderAPI_MakeEdge(P1,P2).Edge()
    MW = BRepBuilderAPI_MakeWire()
    MW.Add(Edge5)
    MW.Add(ExistingWire2)

    if MW.IsDone():
        WhiteWire = MW.Wire()
        LastEdge = MW.Edge()
        LastVertex = MW.Vertex()
		

	return WhiteWire;


if __name__=='__main__':
	#square wave test
	gc = Gcode_Lib.GCode_Generator();
	print '*****Square Wave Test********'
	gc.start();
	gc.followWire( squareWaveWire(),100.0 );
	gc.end();
	print "\n".join(gc.getResults());
	
	gc.reset();
	print
	print '*****Circle Wire Test: ********'
	gc.start();
	gc.followWire( makeSimpleTestWire(),100.0 );
	gc.end();
print "\n".join(gc.getResults());	