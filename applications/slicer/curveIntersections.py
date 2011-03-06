from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
from OCC import Extrema,gp
import time,os,sys,string;
from OCC.Geom import *
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();
from OCC.Utils.Topology import Topo
from OCC.Utils.Topology import WireExplorer

import TestDisplay
import itertools
import Wrappers
import time
import networkx as nx
import cProfile
import pstats
import hexagonlib
import timeit

"""
	Test out intersections of curves.

"""

#make a hexagon wire
h = hexagonlib.Hexagon(0.5,0 );
wirelist = h.makeHexForBoundingBox((0,0,0), (25,25,0));
w = wirelist[0];
TestDisplay.display.showShape(w);


#make square wire
p1 = gp.gp_Pnt(5,-5,0);
p2 = gp.gp_Pnt(20,-5,0);
p3 = gp.gp_Pnt(20,10,0);
p4 = gp.gp_Pnt(5,10,0);
e1 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2).Edge();
e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p2,p3).Edge();
e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p3,p4).Edge();
e4 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p4,p1).Edge();
mw = BRepBuilderAPI.BRepBuilderAPI_MakeWire();
mw.Add(e1);
mw.Add(e2);
mw.Add(e3);
mw.Add(e4);
w2 = mw.Wire();
TestDisplay.display.showShape(w2);


def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
	p.print_stats(level);	
	
def loopWire(w):
	topexp = TopExp.TopExp_Explorer();
	topexp.Init(w,TopAbs.TopAbs_EDGE);
	edges = [];
	while topexp.More():
		currentEdge = Wrappers.cast(topexp.Current());
		edges.append(currentEdge);
		topexp.Next();
	return edges;
	
def loopWire2(w):
	edges = TopTools.TopTools_HSequenceOfShape();
	topexp = TopExp.TopExp_Explorer();
	topexp.Init(w,TopAbs.TopAbs_EDGE);

	while topexp.More():
		#currentEdge = Wrappers.cast();
		edges.Append(topexp.Current());
		topexp.Next();
	return edges;
	
runProfiled('loopWire2(w)',0.9);
t = Wrappers.Timer();
for i in range(1,100):
	loopWire2(w);
print t.elapsed() * 1000;

TestDisplay.display.run();	
