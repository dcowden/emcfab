"""
	Testing 2d and 3d operations -- hopefully 2d code is much faster
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
from OCC import GeomAPI
from OCC import BRepExtrema
from OCC import Geom,Geom2dAPI
from OCC import BRepBndLib 
from OCC import Bnd;
from OCC import BRep;
from OCC import TopTools
from OCC import TopoDS
from OCC import TopAbs
from OCC import TopExp
from OCC import BRepTools
from OCC import TColgp
from OCC import Poly
from OCC import TopLoc
from OCC import BRepMesh
from OCC import GCE2d,BndLib,Geom2dAdaptor,Geom2dAPI
from OCC.Utils.Topology import Topo
from OCC import BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor,IntCurvesFace,Approx,BRepLib,TopTools
import pixmaptest
import Wrappers
import TestWires
import bresenham
import hexagonlib2d

from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()

import pixMapTileTest
import breshamtest as bres
#import bresenham as bres
import numpy as np
import networkx as nx
import hexagonlib
import hexagonlib2d

brt = BRep.BRep_Tool()

def tP(x,y):
	"convert x-y tuple to a gp pnt"
	return gp.gp_Pnt2d(x,y);
	
def edgeFromTwoPoints(p1,p2):
	"make a linear edge from two 2d points. p1 and p2 are simple (x,y) tuples "
	builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge2d(tP(p1[0],p1[1]),tP(p2[0],p2[1]));
	builder.Build();
	if builder.IsDone():
		return builder.Edge();
	else:
		print "error making edge"
		return None;

def makeHeartWire2d():
	"make a heart wire in 2d"
	e1 = edgeFromTwoPoints((0,0),(4.0,4.0));
	
	circle = gp.gp_Circ2d(gp.gp_Ax2d(tP(2,4),gp.gp().DX2d() ),2);
	e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge2d(circle, tP(4,4),tP(0,4)).Edge();
	
	circle = gp.gp_Circ2d(gp.gp_Ax2d(tP(-2,4),gp.gp().DX2d()),2);
	e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge2d(circle, tP(0,4),tP(-4,4)).Edge();
	
	e4 = edgeFromTwoPoints((-4,4), (0,0) );
	return Wrappers.wireFromEdges([e1,e2,e3,e4]);

def makeHeartWire():
	"make a heart wire"
	e1 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(0,0,0), gp.gp_Pnt(4.0,4.0,0));
	circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(2,4,0),gp.gp().DZ()),2);
	e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle, gp.gp_Pnt(4,4,0),gp.gp_Pnt(0,4,0)).Edge();
	circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(-2,4,0),gp.gp().DZ()),2);
	e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle, gp.gp_Pnt(0,4,0),gp.gp_Pnt(-4,4,0)).Edge();
	e4 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(-4,4,0), gp.gp_Pnt(0,0,0));
	return Wrappers.wireFromEdges([e1,e2,e3,e4]);	
	
"""
	Interesting-- i thought this would be slower than BREpBuilder_MakeEdge2d, but it really wasnt.
"""
def makeFillCurves2d(xMin,yMin,xMax,yMax,spacing):
	""" 
		makes a set of lines that are curves not edges
		probably the minimal possible construction
	"""
	lines = [];
	for y in Wrappers.frange6(yMin,yMax,spacing):
		l = GCE2d.GCE2d_MakeSegment(tP(xMin,y),tP(xMax,y) ).Value()
		lines.append(l)
	return lines;

def makeSingle2dFillCurve(xMin,yMin,xMax,yMax):
	return [ GCE2d.GCE2d_MakeSegment(tP(xMin,yMin),tP(xMax,yMax)).Value() ];
	
def makeFillEdges2d(xMin,yMin,xMax,yMax, spacing):
	"make straight hatch lines."
	edges = [];
	for y in Wrappers.frange6(yMin,yMax,spacing):
		e = edgeFromTwoPoints((xMin,y),(xMax,y));
		#TestDisplay.display.showShape(e);
		edges.append(e);
	return edges;

"""

	offset a 2d wire
"""
def offset2dWire(wire, distance ):
	bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
	bo.AddWire(wire);

	bo.Perform(distance,0.0);  #this line crashes hard, but only sometimes.
	#print "done offsetting..";
	if  not bo.IsDone():
		raise Exception, "Offset Was Not Successful.";
	else:
		return  bo.Shape();	

def get2dCurveFrom3dEdge(edge):
	"""
		returns a curve given an edge.
		here, we want to get a curve from a 3dEdge, since with this approach we'll only be getting existing
		curves from a 3d source
	"""
	#first, convert the curve to a 2d curve
	btool = BRep.BRep_Tool();
	handleCurve = btool.Curve(edge)[0]
	return  GeomAPI.GeomAPI().To2d(handleCurve, gp.gp_Pln( gp.gp_Pnt(0,0,0), gp.gp().DZ() ) )	


	
"""
	Finds intersections between trimmed durves
"""
def findIntersectionsUsingCurves(wire,lines):
	points = []
	count = 0
	w = Wrappers.Wire(wire)
	for boundaryEdge in w.edges2():
		#get the curve from the 2d edge
		hCurve = get2dCurveFrom3dEdge(boundaryEdge)
		curve = hCurve.GetObject();
		a = gp.gp_Pnt2d()
		b = gp.gp_Pnt2d()
		for l in lines:
			#the line is already a geom-curve
			lc = l.GetObject()
			count+=1;
			isector = Geom2dAPI.Geom2dAPI_ExtremaCurveCurve(hCurve,l,curve.FirstParameter(),curve.LastParameter(), lc.FirstParameter(), lc.LastParameter() );
			if isector.NbExtrema() > 0 :
				isector.NearestPoints(a,b)
				points.append( a );
	return (count,points);
	
def findIntersectionsUsingCurvesNoSorting(wire,lines):
	"""
		find all of the intersections points between the wire and the lines, 
		no fancy sorting
	"""
	points = []
	count = 0;
	w = Wrappers.Wire(wire)
	for boundaryEdge in w.edges2():
		#use this code if the supplied wire is a 3d wire
		#get the curve from the 2d edge
		#hCurve = get2dCurveFrom3dEdge(boundaryEdge)
		#curve=hCurve.GetObject();
		#lp = curve.FirstParameter();
		#up = curve.LastParameter();
		
		#use this code if the supplied wire is a wire of 2d edges
		r = brt.Curve(boundaryEdge);
		hCurve = r[0];
		lp = r[1];
		up = r[2];
		print hCurve

		for l in lines:
			#the line is already a geom-curve
			count+=1;
			isector = Geom2dAPI.Geom2dAPI_InterCurveCurve(hCurve,l );
			if isector.NbPoints() > 0:
				for k in range(1,isector.NbPoints()+1):
					#make sure the point is actually within the parameter boundaries on the curve.
					(a,b) = projectPointOnCurve2d(hCurve,isector.Point(k),0.001);
					if a >= lp and a <= up:
						points.append( isector.Point(k) );
	return (count,points);

def boundingBoxForCurve(curve):
	box = Bnd.Bnd_Box2d();
	BndLib.BndLib_Add2dCurve().Add(Geom2dAdaptor.Geom2dAdaptor_Curve(curve),0.001, box);
	return box;

def projectPointOnCurve2d(handleCurve,point,tolerance):
	"return the closest parameter on the curve for the provided point"
	"returns none if a point is not within tolerance"
	
	#get start and end params of curve
	curve = handleCurve.GetObject()

	gp = Geom2dAPI.Geom2dAPI_ProjectPointOnCurve(point,handleCurve,curve.FirstParameter(), curve.LastParameter());
	if gp.NbPoints()>0 and gp.LowerDistance() <= tolerance:
		#log.debug( "Projection Success!" );
		#print "project success."
		return (gp.LowerDistanceParameter(),gp.NearestPoint())
	else:
		print( "Projection Failed.")
		return None;
		
def testProjectingPointInaccurately():
	"""
		test projecting a point onto a curve
	"""
	h = makeHeartWire();
	
	#convert to twod curves
	curves = []
	for e in Wrappers.Wire(h).edges2():
		curves.append (get2dCurveFrom3dEdge(e) );
	
	#project a point onto the wire. we want to see if slight inaccurcies will project.
	#this simulates trying to find points on a curve that were put there via pixels
	display.DisplayShape(h);
	DISTANCE = 0.004
	p = gp.gp_Pnt2d(2.0+DISTANCE,2.0-DISTANCE);
	display.DisplayShape( Wrappers.make_vertex(gp.gp_Pnt(p.X(),p.Y(),0)));
	for c in curves:
		r = projectPointOnCurve2d(c,p,0.005);
		if r is not None:
			(param,pnt) = r
			print "found point: parmater-%0.3f, point-(%0.3f,%0.3f)" % ( param, pnt.X(), pnt.Y() );
			display.DisplayShape( Wrappers.make_vertex(gp.gp_Pnt(pnt.X(),pnt.Y(),0)));
	
	
	
def findIntersectionsUsingCurvesWithBoundingBoxes(wire,lines):
	"""
		find all of the intersections points between the wire and the lines, 
		no fancy sorting
	"""
	points = []
	count = 0;
	rejected=0;
	w = Wrappers.Wire(wire)
	for boundaryEdge in w.edges2():
		#get the curve from the 2d edge
		curve = get2dCurveFrom3dEdge(boundaryEdge)
		#make bounding box for the test curve
		box = boundingBoxForCurve(curve)

		for l in lines:
			#the line is already a geom-curve-- 
			count+=1;
			
			box2 = boundingBoxForCurve(l)
			if box2.IsOut(box):
				rejected+=1;
				continue;
			isector = Geom2dAPI.Geom2dAPI_InterCurveCurve(curve,l );
			if isector.NbPoints() > 0:
				for k in range(1,isector.NbPoints()+1):
					points.append( isector.Point(k) );
	print "Rejected %d intersections." % rejected
	return (count,points);	


	
def displayCurve(curve):
	"""
		Displays a curve. kind of a pain, since curves must be converted to edges first
	"""
	edge = BRepBuilderAPI.BRepBuilderAPI_MakeEdge2d(curve).Edge()
	display.DisplayShape( edge,update=False)

def displayCurveList(wire):
	for c in wire:
		displayCurve(c)
def intersectWiresUsingDistShapeShape(wire,edges):
	"intersect a wire with a series of edges. naive algorithm without bounding box sorting "
	ipoints = []
	w = Wrappers.Wire(wire);
	
	circle = gp.gp_Circ2d(gp.gp_Ax2d(tP(2,4),gp.gp().DX2d() ),2);
	e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge2d(circle, tP(4,4),tP(0,4)).Edge();
	TestDisplay.display.showShape(e2);
	e4 = edgeFromTwoPoints((-4,4), (0,0) );
	TestDisplay.display.showShape(e4);
	brp = BRepExtrema.BRepExtrema_DistShapeShape(e2,e4);
	print "runing"
	brp.Perform()
	print "done"
	if brp.Value() < 0.001:
		print "intersection found!"
		#TODO need to handle the somewhat unusual cases that the intersection is
		#on a vertex
		for k in range(1,brp.NbSolution()+1):
			p1 = brp.PointOnShape1(k);
			ipoints.append ( p1.X(), p1.Y() );


	return (count,ipoints);
	
"""
	Surprising-- making 2d edges is faster than 2d curves!
	plus curves are a lot less convienient because they cannot be used to display like edges can be!
"""
if __name__=='__main__':
	h2 = makeHeartWire2d();
	h3 = makeHeartWire();
	#make 2d fill lines using MakeEdge2d
	q = time.clock();
	fillEdges = makeFillEdges2d(-5,0,5,8,0.01 );
	#for e in fillEdges:
	#	display.DisplayShape(e );
	print ("Made 500 edges in %0.3f" ) % ( time.clock() - q);
	
	#make 2d fill using 2d curves
	q = time.clock();
	fillCurves = makeFillCurves2d(-5,0,5,8,0.1);
	print ("Made %d curves in %0.3f" ) % (len(fillCurves), time.clock() - q);
	#disaply the fill curves
	for f in fillCurves:
		displayCurve(f);
		
	display.DisplayShape(h2);
	
	#make some hexagons
	hex = hexagonlib2d.Hexagon(0.250,0.01);
	hexarray = hex.makeHexForBoundingBox((-5,0),(5,7) );
	
	q = time.clock();
	testProjectingPointInaccurately();
	print ("Computed  in %0.3f" ) % ( (time.clock() - q));

	
	#flatten all of these edges into one list
	#allHexLines = []
	#for a in hexarray:
	#	allHexLines.extend(a);
		
	#fine all intersections using curves directly,  with no sorting or boudning boxes
	#q = time.clock();
	#(count,ipoints) = findIntersectionsUsingCurvesNoSorting(h3,fillCurves);
	#print ("Computed %d intersections in %0.3f: %d intersections found" ) % ( count, (time.clock() - q),len(ipoints));

	q = time.clock();
	#(count,ipoints) = findIntersectionsUsingCurves(h3,fillCurves);
	(count,ipoints) = findIntersectionsUsingCurvesNoSorting(h2,fillCurves);
	print ("Computed %d intersections in %0.3f: %d intersections found" ) % ( count, (time.clock() - q),len(ipoints));

	for p in ipoints:
		display.DisplayShape( Wrappers.make_vertex ( gp.gp_Pnt( p.X(), p.Y(), 0 ) ) , update=False); #annoying-- no 2d vertex exists really
	#display wire
	print "Done"
	#for p in ipoints:
	#	display.DisplayShape(Wrappers.make_vertex(gp.gp_Pnt(p.X(),p.Y(),0)) ,update=False );
	#displayCurveList(fillCurves);
	
#	for w in fillWires:
#		TestDisplay.display.showShape(w);
	#display.DisplayShape(h3);
	display.FitAll();
	#TestDisplay.display.showShape(offset2dWire(h,-0.1))
	start_display()
