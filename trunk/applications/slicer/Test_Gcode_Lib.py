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
	Z=0;
	pts = [[0,0,Z],[1,0,Z],[1,1,Z],[2,1,Z],[3,3,Z]];
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
    #       The red wire is build from a single edge
    Elips = gp_Elips(gp_Ax2(gp_Pnt(250,0,0),gp_Dir(1,1,1)),160,90)
    Edge1 = BRepBuilderAPI_MakeEdge(Elips,0,math.pi/2).Edge()
    RedWire = BRepBuilderAPI_MakeWire(Edge1).Wire()
    ###       the yellow wire is build from an existing wire and an edge
    circle = gp_Circ(gp_Ax2(gp_Pnt(-300,0,0),gp_Dir(1,0,0)),80)
    Edge2 = BRepBuilderAPI_MakeEdge(circle,0,math.pi).Edge()
    ExistingWire = BRepBuilderAPI_MakeWire(Edge2).Wire()
    Edge3 = BRepBuilderAPI_MakeEdge(gp_Pnt(-300,0,-80),gp_Pnt(-90,20,-30)).Edge()
    MW1 = BRepBuilderAPI_MakeWire(ExistingWire,Edge3)
    ##print "1"
    if MW1.IsDone():
        YellowWire = MW1.Wire()
    ###       the white wire is built with an existing wire and 3 edges.
    ###       we use the methods Add, Edge and Vertex from BRepBuilderAPI_MakeWire.
    circle2 = gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,1,0)),200)
    Edge4 = BRepBuilderAPI_MakeEdge(circle2,0,math.pi).Edge()
    ExistingWire2 = BRepBuilderAPI_MakeWire(Edge4).Wire()
    P1 = gp_Pnt(0,0,-200)
    P2 = gp_Pnt(5,0,0) #5,204,0
    Edge5 = BRepBuilderAPI_MakeEdge(P1,P2).Edge()
    P3 = gp_Pnt(-15,0,15)
    Edge6 = BRepBuilderAPI_MakeEdge(P2,P3).Edge()
    P4 = gp_Pnt(15,0,0)
    Edge7 = BRepBuilderAPI_MakeEdge(P3,P4).Edge()
    MW = BRepBuilderAPI_MakeWire()
    MW.Add(ExistingWire2)
    MW.Add(Edge5)
    MW.Add(Edge6)
    MW.Add(Edge7)
    if MW.IsDone():
        WhiteWire = MW.Wire();
		
	return WhiteWire;


if __name__=='__main__':
	#square wave test
	gc = Gcode_Lib.GCode_Generator();
	print '*****Square Wave Test********'
	print "\n".join(gc.start());
	print "\n".join(gc.followWire( squareWaveWire(),100.0 ));
	print "\n".join(gc.end());
	
	gc.reset();
	print
	print '*****Circle Wire Test: ********'
	print "\n".join(gc.start());
	print "\n".join(gc.followWire( makeSimpleTestWire(),100.0 ));
	print "\n".join(gc.end());	