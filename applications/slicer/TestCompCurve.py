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

def listFromArray( stdArray ):
	results= []
	for i in range(stdArray.Lower(),stdArray.Upper() ):
		results.append(stdArray.Value(i));
	return results;

if __name__=='__main__':
	
	#make test wire, which has two edges
	[testWire,circleEdge,lineEdge] = TestDisplay.makeKeyHoleWire();
	testWire2 = TestDisplay.makeSquareWire();
	
	if testWire2.Closed():
		print "testWire2 is closed";
	else:
		print "testWire2 is not closed.";
	TOLERANCE = 0.00001;
	curve  = BRepAdaptor.BRepAdaptor_CompCurve (testWire);
	
	print "CompCurve is parameterized between %0.5f and %0.5f " % (  curve.FirstParameter(), curve.LastParameter() );
	print "%d Continuity" % curve.Continuity();
	print "%d Intervals(C0)" % curve.NbIntervals(0);
	print "%d Intervals(C1)" % curve.NbIntervals(1);
	print "%d Intervals(C2)" % curve.NbIntervals(2);
	print "%d Intervals(C3)" % curve.NbIntervals(3);
	
	nB = curve.NbIntervals(2);
	
	#TestDisplay.display.showShape(testWire);
	resultEdge = TopoDS.TopoDS_Edge();
	#try to get edge from paramter
	p = 0.5;
	eP = curve.Edge(p,resultEdge);
	print "CompCurve Parameter %0.2f = edge parameter %0.2f" % ( p, eP) ;
	TestDisplay.display.showShape(resultEdge);
	
	#show the selected point also
	point = curve.Value(p);
	
	#get all of the intervals for the wire
	print "Getting %d Intervals" % nB;
	array = TColStd.TColStd_Array1OfReal(1,nB+2);
	
	curve.Intervals(array,2);
	print "Bounding Parameters: ",listFromArray(array);
	TestDisplay.display.showShape(Wrappers.make_vertex(point));
	
	
	wr = Wrappers.Wire(testWire);
	edgeList = []
	for e in wr.edges2():
		edgeList.append(e);
		
	print "Edges:",edgeList
	
	
	
	TestDisplay.display.run();