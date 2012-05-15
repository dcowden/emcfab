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
from OCC import BRep;
from OCC import TopTools
from OCC import TopoDS
from OCC import TopAbs
from OCC import TopExp
from OCC import BRepTools
from OCC import TColgp
from OCC import Poly
from OCC import TopLoc
from OCC import BRepMesh,BRepClass
from OCC.Utils.Topology import Topo
from OCC import BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor,IntCurvesFace,Approx,BRepLib,TopTools
import pixmaptest
import Wrappers
import TestWires
import TestDisplay
import pixMapTileTest
import breshamtest as bres
#import bresenham as bres
import numpy as np
import networkx as nx
import hexagonlib

ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();
brt = BRepTools.BRepTools();

def tP(pnt):
	return (pnt.X(),pnt.Y());
def pnt(tuple):
	#print "pnt:",tuple;
	return gp.gp_Pnt((float)(tuple[0]),(float)(tuple[1]),0.0);
	
def makeHeartWire():
	"make a heart wire"
	e1 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(0,0,0), gp.gp_Pnt(4.0,4.0,0));
	circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(2,4,0),gp.gp().DZ()),2);
	e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle, gp.gp_Pnt(4,4,0),gp.gp_Pnt(0,4,0)).Edge();
	circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(-2,4,0),gp.gp().DZ()),2);
	e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle, gp.gp_Pnt(0,4,0),gp.gp_Pnt(-4,4,0)).Edge();
	e4 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(-4,4,0), gp.gp_Pnt(0,0,0));
	return Wrappers.wireFromEdges([e1,e2,e3,e4]);

def makePieWire():
	e1 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(0,0,0), gp.gp_Pnt(4.0,0,0));
	e2 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(4.0,0,0), gp.gp_Pnt(2.0,0.1,0));
	e3 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(2.0,0.1,0), gp.gp_Pnt(3.0,1.0,0));
	e4 = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(3.0,1.0,0), gp.gp_Pnt(00,0,0));
	return Wrappers.wireFromEdges([e1,e2,e3,e4]);	

	
def testOffsetReferences():
	ow = makeHeartWire();
	
	bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
	bo.AddWire(ow);
	TestDisplay.display.showShape(ow )
	bo.Perform(-0.01,0.0);
	
	newWire = bo.Shape();
	TestDisplay.display.showShape(newWire);
	
	#now see if we can figure out which edge in the new wire goes with which
	for oldEdge in Topo(ow).edges():
		listOfShape = bo.Generated(oldEdge); #listOfShape is TopTools.TopTools_ListOfShape
		newEdge = listOfShape.First();
		
		print "Old EdgeID is %d, New EdgeId is %d" % ( oldEdge.__hash__(), newEdge.__hash__()   );
		
#offset a face, returning the offset shape
def offsetFace(face,offset ):
	ow = brt.OuterWire(face);
	bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
	bo.AddWire(ow);
	
	
	for w in Topo(face).wires():
		if not w.IsSame(ow):
			#TestDisplay.display.showShape(w);
			bo.AddWire(w);
	
	#print "about to offset by %0.2f" % offset;
	bo.Perform(offset,0.0);  #this line crashes hard, but only sometimes.
	#print "done offsetting..";
	bo.Check()
	return bo.Shape()
	
def makeCircleWire():
	circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(0,2,0),gp.gp().DZ()),.75);
	e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle, gp.gp_Pnt(0,1.25,0),gp.gp_Pnt(0,1.25,0)).Edge();
	return Wrappers.wireFromEdges([e2]);

def boundingBox(shapeList):
	"get a bounding box for a list of shapes"
	box = Bnd.Bnd_Box();
	b = BRepBndLib.BRepBndLib();	

	for s in shapeList:
		b.Add(s,box);
		
	bounds = box.Get();
	return bounds; #(xMin,yMin,zMin,xMax,yMax,zMax)

def scanlinesFromBoundingBox(boundingBox,interval):
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox;
	print boundingBox;
	edges = [];
	for y in Wrappers.frange6(yMin,yMax,interval):
		e = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(xMin,y,0),gp.gp_Pnt(xMax,y,0));
		#TestDisplay.display.showShape(e);
		edges.append((y,Wrappers.wireFromEdges([e])));
	return edges;

	
def triangulateAtDeflection(shape,deflection):
	b = BRepMesh.BRepMesh();
	b.Mesh(shape,deflection);
	

def boundarypixmapFromFace(face):
	"""	
		creates a pixel map containing only the boundaries of the object.
		the shape is approximated with straight lines, and vertices are marked
		with different values to help re-construct the lines later on.	
		
		Some c++ optimizaion woudl really speed this up.
	"""
	PIXEL = 0.02;
	DEFLECTION = PIXEL / 4.0;

	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([face]);
		
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin,yMin),(xMax,yMax),PIXEL);

	#approximate each wire with a set of segments
	bb = TopExp.TopExp_Explorer();
	bb.Init(face,TopAbs.TopAbs_WIRE);
	while bb.More():
		#print "plotting wire"
		w = Wrappers.Wire(Wrappers.cast(bb.Current()));
		
		#divide the edge into discrete segments
		lastPnt = None;
		for e in w.edges2():
			#for each edge, set vertices, and compute points on the edge
			ew = Wrappers.Edge(e);
			lastPnt = None;
			for pnt in ew.discretePoints(DEFLECTION):				
				pixmap.set(tP(pnt),2);
						
				#plot the line
				if lastPnt != None: pixmap.drawLine(tP(lastPnt),tP(pnt) );				
				lastPnt = pnt;
		bb.Next();
	
	return pixmap;

def tuplesFromEdge(edge,deflection):
	ew = Wrappers.Edge(edge);
	list = [];
	for p in ew.discretePoints(deflection):
		list.append(tP(p));
	return list;

def tuplesFromWire(wire,deflection):
	"return a list of tuples from a wire"
	list = [];
	ww = Wrappers.Wire(wire);
	for e in ww.edges2():
		
		#for each edge, set vertices, and compute points on the edge
		ew = Wrappers.Edge(e);
		for pnt in ew.discretePoints(deflection):				
			list.append(tP(pnt));
	return list;

def makeHexEdges(xMin,yMin,xMax,yMax,zLevel):
	#this can be faster by avoiding 3d wires in the first place, for for now it is ok
	hex = hexagonlib.Hexagon(0.35,0.01);
				
	wires = hex.makeHexForBoundingBox(( xMin,yMin,zLevel),(xMax,yMax,zLevel ));
	tt = [];
	for w in wires:
		ww = Wrappers.Wire(w);
		for e in ww.edges2():
			ew = Wrappers.Edge(e);
			tt.append((tP(ew.firstPoint),tP(ew.lastPoint)));

	print "Made %d hex edges" % len(tt)
	return tt;
	
def drawWire( tupleList, pixmap, fillColor=1 ):
	"draws a wire onto an image"
	for et in Wrappers.pairwise(tupleList):
		pixmap.drawLine(et[0],et[1],fillColor);	

def trimEdges(edgeList, pixmap):
	"accepts a list of tuples ( each represents an edge)"
	"returns a networkx network of points, based on edges in the pixmap"
	"the network is in pixel coordinates ( integers )"
	
	g = nx.Graph();
	errors = 0;
	for e in edgeList:
		#print "Trimming Edge",e
		try:
			"walk each pixel, trimming it to segments inside the surface"
			start = pixmap.index(e[0]);
			end = pixmap.index(e[1]);
			#print e,start,end
			result_edges = bres.piecewise_bresenham_line(pixmap.p,start,end );
			for re in result_edges:
				"each of these are on the map"
				g.add_edge(re[0],re[1] );
		except:
			errors += 1;
			#print "error trimming."
			#raise;
	
	print "Errors Trimming %d edges" % errors;
	return g;

def displayGraph(graph):
	"display an networkx graph"
	"this is just a hack-- the tuples are in integer coordinates, and this will be terribly slow"
	
	for e in graph.edges_iter():
		try:
			TestDisplay.display.showShape(Wrappers.edgeFromTwoPoints(pnt(e[0]),pnt(e[1])));
		except:
			pass;

			
def testWirePixmap(face):
	"""
		tests drawing a wire while adjusting sharp borders
	"""
	PIXEL = 0.01 ;
	DEFLECTION = PIXEL / 4.0;

	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([face]);
	g = nx.Graph();
	#adjust boundaries a bit
	BUFFER=PIXEL*5;
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin-BUFFER,yMin-BUFFER),(xMax+BUFFER,yMax+BUFFER),PIXEL);

	
	ow = brt.OuterWire(face);
	boundary = tuplesFromWire(ow,DEFLECTION);
	
	for et in Wrappers.pairwise(boundary):
		p1 = et[0];
		p2 = et[1];
		g.add_edge(p1,p2);

		i1 = pixmap.index(p1);
		i2 = pixmap.index(p2);
		#print i1,i2,p1,p2
		lines = bres.piecewise_bresenham_line(pixmap.p,i1,i2);
		
		#add to the graph. this is tricky. we want to add the original locations
		#to the graph if nothing changed, but we want to add the new locations
		#to the graph is something did change or new points were added
		for l in lines:
			#print "adding edge."
			g.add_edge(l[0],l[1]);
	
	return g;

def testWirePixmap2(face):
	"""
		tests drawing a wire while adjusting sharp borders
	"""
	PIXEL = 0.01 ;
	DEFLECTION = PIXEL / 4.0;

	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([face]);
	g = nx.Graph();
	#adjust boundaries a bit
	BUFFER=PIXEL*5;
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin-BUFFER,yMin-BUFFER),(xMax+BUFFER,yMax+BUFFER),PIXEL);

	
	ow = brt.OuterWire(face);
	boundary = tuplesFromWire(ow,DEFLECTION);
	
	for et in Wrappers.pairwise(boundary):
		p1 = et[0];
		p2 = et[1];
		g.add_edge(p1,p2);

		i1 = pixmap.index(p1);
		i2 = pixmap.index(p2);
		#print i1,i2,p1,p2
		pixmap.drawLine(p1,p2,1,2);
	
	pixmap.saveImage("c:\\temp\\thickboundary.jpg");


def testHugePixmap(face):
	"""
		tests drawing a wire while adjusting sharp borders
	"""
	PIXEL = 0.005 ;
	DEFLECTION = PIXEL / 2.0;

	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([face]);
	g = nx.Graph();
	#adjust boundaries a bit
	BUFFER=PIXEL*5;
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin-BUFFER,yMin-BUFFER),(xMax+BUFFER,yMax+BUFFER),PIXEL);

	
	ow = brt.OuterWire(face);
	boundary = tuplesFromWire(ow,DEFLECTION);
	
	for et in Wrappers.pairwise(boundary):
		p1 = et[0];
		p2 = et[1];
		g.add_edge(p1,p2);

		i1 = pixmap.index(p1);
		i2 = pixmap.index(p2);
		#print i1,i2,p1,p2
		pixmap.drawLine(p1,p2,1,2);
	
	#now, lets see how long it takes to find the intersection of a line by painting on the pixmap
	q = time.clock();
	p1 = ( -4.0,2.0 )
	p2 = ( 4.0, 2.0 )
	results = pixmap.drawLine2(p1,p2 )
	print results
	print "Found Intersections in %0.3f seconds" % ( time.clock() - q );
	
	#how fast is it to determine if a point is on the face?
	q = time.clock()
	p1 = ( -0.5,2.0)
	for i in range(1,1000):
		pixmap.get(p1)
	print "Tested 1000 points in %0.3f seconds" % ( time.clock() - q );
	pixmap.saveImage( "c:\\temp\scanline.jpg" );

def pixmapFromWires(outerWire,innerWireList,fillpattern=None):
	"""
		create a filled pixmap from a defined face.
		
		ideally, this is really really fast, based on a triangulation of the face.
		a face with 800x600 pixels took 56ms to raster with this algo. @ 0.012" that is a 6x8 " part!

		this creates a pixel map that has all the interior pixels filled
		
		This version uses the boundaries and PIL.polygon instead of 
		the triangulagion, primarily becaseu the OCC triangulation seems to suck
		
		the general approach is:
			compute a filled shape containging a bit mask
			draw borders using a value
			tile a fill pattern onto the background
	"""
	
	PIXEL = 0.010 ;
	DEFLECTION = PIXEL / 2.0;

	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([outerWire]);
	
	#adjust boundaries a bit
	BUFFER=PIXEL*5;
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin-BUFFER,yMin-BUFFER),(xMax+BUFFER,yMax+BUFFER),PIXEL);

	#we will draw the outer wire as a filled polygon, then
	#draw the inner wire as another filled polygon
	
	outerPoints = tuplesFromWire(outerWire,DEFLECTION);
	#outerPoints.pop(2);
	#print outerPoints
	pixmap.drawPolygon(outerPoints,1,1);
	#drawWire(outerPoints,pixmap);
	#pixmap.saveImage("c:\\temp\\heartborder-1.bmp");
	
	#get the other wires.
	wires = []
	wires.append(outerPoints);
	for w in innerWireList:
		wp = tuplesFromWire(w,DEFLECTION);
		pixmap.drawPolygon(wp,0,0);
		wires.append(wp);
			
	return pixmap;

def pixmapFromFace2(face,fillpattern=None):
	"""
		create a filled pixmap from a defined face.
		
		ideally, this is really really fast, based on a triangulation of the face.
		a face with 800x600 pixels took 56ms to raster with this algo. @ 0.012" that is a 6x8 " part!

		this creates a pixel map that has all the interior pixels filled
		
		This version uses the boundaries and PIL.polygon instead of 
		the triangulagion, primarily becaseu the OCC triangulation seems to suck
		
		the general approach is:
			compute a filled shape containging a bit mask
			draw borders using a value
			tile a fill pattern onto the background
	"""
	
	PIXEL = 0.012 ;
	DEFLECTION = PIXEL / 2.0;

	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([face]);
	
	#adjust boundaries a bit
	BUFFER=PIXEL*5;
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin-BUFFER,yMin-BUFFER),(xMax+BUFFER,yMax+BUFFER),PIXEL);

	#we will draw the outer wire as a filled polygon, then
	#draw the inner wire as another filled polygon
	
	ow = brt.OuterWire(face);
	outerPoints = tuplesFromWire(ow,DEFLECTION);
	#outerPoints.pop(2);
	#print outerPoints
	pixmap.drawPolygon(outerPoints,1,1);
	#drawWire(outerPoints,pixmap);
	#pixmap.saveImage("c:\\temp\\heartborder-1.bmp");
	
	#get the other wires.
	wires = []
	wires.append(outerPoints);
	for w in Topo(face).wires():
		if not w.IsSame(ow):
			wp = tuplesFromWire(w,DEFLECTION);
			pixmap.drawPolygon(wp,0,0);
			wires.append(wp);
			
	return pixmap;

def testPointInFacePerformance(face):
	"""
		tests how quickly we can dtermine if a point is on a face
	"""
	pnt = gp.gp_Pnt2d(0.5,2.0);
	TOLERANCE = 0.001;
	q = time.clock()
	for i in range(1000):
		bf = BRepClass.BRepClass_FaceExplorer(face)
		result = bf.Reject(pnt)
	print ("Computed point in face 100 times in %0.3f" % ( time.clock() - q ) )
	
def pixmapFromFaceTriangulation(face):
	"""
		create a filled pixmap from a defined face.
		
		ideally, this is really really fast, based on a triangulation of the face.
		a face with 800x600 pixels took 56ms to raster with this algo. @ 0.012" that is a 6x8 " part!

		this creates a pixel map that has all the interior pixels filled
		
		
		this might fix:
			TopoDS_Face tFace = BRepBuilderAPI_MakeFace( tFrame );
			BRepBuilderAPI_MakeFace     MF( tFace );
			MF.Add( tHole );
			if ( MF.IsDone()) {
			Handle(ShapeFix_Shape) sfs = new ShapeFix_Shape;
			sfs->Init ( MF.Shape() );
			sfs->Perform();
			Handle(AIS_Shape) aShape = new AIS_Shape(sfs->Shape());
			myAISContext->Display(aShape);		
	"""
	
	PIXEL = 0.005;
	DEFLECTION = PIXEL / 20.0;
	triangulateAtDeflection(face,DEFLECTION);
	
	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([face]);
	
	#adjust boundaries a bit
	BUFFER=PIXEL*5;
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin-BUFFER,yMin-BUFFER),(xMax+BUFFER,yMax+BUFFER),PIXEL);
	
	transformPoints = True;
	loc = TopLoc.TopLoc_Location();
	hT = btool.Triangulation(face,loc).GetObject();
			
	trsf = loc.Transformation();
	
	if loc.IsIdentity():
		transformPoints = False;

		nodes = hT.Nodes();
	triangles = hT.Triangles();
	
	#mark pixels to fill
	for i in range(triangles.Lower(),triangles.Upper() ):
		tr = triangles.Value(i);
		(n1,n2,n3) = tr.Get();
		p1 = nodes.Value(n1);
		p2 = nodes.Value(n2);
		p3 = nodes.Value(n3);
		
		if transformPoints:
			p1.Transform(trsf);
			p2.Transform(trsf);
			p3.Transform(trsf);
	
		#fill triangle. easy peasy!
		pixmap.fillTriangle(tP(p1),tP(p2),tP(p3),7);
		#TestDisplay.display.showShape(Wrappers.edgeFromTwoPoints(p1,p2));
		#TestDisplay.display.showShape(Wrappers.edgeFromTwoPoints(p2,p3));
		#TestDisplay.display.showShape(Wrappers.edgeFromTwoPoints(p3,p1));
		#time.sleep(1);
		#TestDisplay.display.showShape(Wrappers.make_vertex(p1));
		#TestDisplay.display.showShape(Wrappers.make_vertex(p2));
		#TestDisplay.display.showShape(Wrappers.make_vertex(p3));
		
	#tile hex pattern onto the filling
	#pixmap.tileOnto(fillpattern);
	
	#mark boundaries
	#approximate each wire with a set of segments
	"""
	bb = TopExp.TopExp_Explorer();
	bb.Init(face,TopAbs.TopAbs_WIRE);
	edgeNum = 1;
	#while False:
	while bb.More():
		#print "plotting wire"
		w = Wrappers.Wire(Wrappers.cast(bb.Current()));
		
		#divide the edge into discrete segments
		lastPnt = None;
		for e in w.edges2():
			
			#for each edge, set vertices, and compute points on the edge
			ew = Wrappers.Edge(e);
			lastPnt = None;
			for pnt in ew.discretePoints(DEFLECTION):				
				pixmap.set(tP(pnt),7);
				edgeNum += 1;
				#plot the line
				#if lastPnt != None: pixmap.drawLine(tP(lastPnt),tP(pnt),edgeNum );				
				lastPnt = pnt;
		bb.Next();
	"""
	return pixmap;
	
#syntax example to tile
"""
	>>> a = np.array([1,0, 1,0,1,0,1,0,1]).reshape(3,-1);
	>>> a
	array([[1, 0, 1],
		   [0, 1, 0],
		   [1, 0, 1]])
	>>> z = np.tile(a,(3,3));
	>>> z
	array([[1, 0, 1, 1, 0, 1, 1, 0, 1],
		   [0, 1, 0, 0, 1, 0, 0, 1, 0],
		   [1, 0, 1, 1, 0, 1, 1, 0, 1],
		   [1, 0, 1, 1, 0, 1, 1, 0, 1],
		   [0, 1, 0, 0, 1, 0, 0, 1, 0],
		   [1, 0, 1, 1, 0, 1, 1, 0, 1],
		   [1, 0, 1, 1, 0, 1, 1, 0, 1],
		   [0, 1, 0, 0, 1, 0, 0, 1, 0],
		   [1, 0, 1, 1, 0, 1, 1, 0, 1]])
>>>
"""

def faceFromWires(outer, innerWireList):
	"make a face from a set of wires"
	builder = BRepBuilderAPI.BRepBuilderAPI_MakeFace(outer);
	for w in innerWireList:
		builder.Add(w);
	builder.Build();
	f = builder.Face();
	#TestDisplay.display.showShape(f);
	return f;
	
def getHeartFace():
	w2 = makeCircleWire();
	w = makeHeartWire();
	return faceFromWires(w, [w2]);

if __name__=='__main__':

	w = makeHeartWire();
	w2 = makeCircleWire();
	f = faceFromWires(makeHeartWire(),[w2] );
	q = time.clock();
	testHugePixmap(f);
	print "HugePixMap: Elapsed %0.3f" % ( (time.clock() - q ));
	
	#testOffsetReferences();
	testPointInFacePerformance(f);
	
	#pm = pixmapFromWires([w2,w] );
	#f = faceFromWires(w,[w2]);
	#f = faceFromWires(w,[]);
	f = faceFromWires(makeHeartWire(),[w2] );
	testWirePixmap2(f);
	TestDisplay.display.showShape(f)
	q = time.clock();
	pm = boundarypixmapFromFace(f);
	pm.saveImage("c:\\temp\\boundary3.jpg");
	#for i in range(1,20):
	#	tmp = offsetFace(f, -0.01*i)
	#	TestDisplay.display.showShape(tmp)
	#	if not tmp:
			
	#		break;
	print "BoundaryMapFromFace: Elapsed %0.3f" % ( (time.clock() - q ));
	
	q = time.clock();
	pm = pixmapFromFace2(f);
	print "pixmapFromFace2: Elapsed %0.3f" % ( (time.clock() - q ));	
	pm.saveImage("c:\\temp\\pixmapFromFace2.jpg");
	q = time.clock();
	pm = pixmapFromFaceTriangulation(f);
	print "pixmapFromFaceTriangulation: Elapsed %0.3f" % ( (time.clock() - q ));	
	pm.saveImage("c:\\temp\\pixmapFromFaceTriangulation.jpg");
	
	#print pm.p.shape;
	#print pm.p
	#pm.saveImage("c:\\temp\\pleasework2.jpg");

	#overlay a hex tiling
	#a small hex tile
	#weird rule needs to change later: vertices must be 2x the valueof the path they are on,
	#so that it is possible to know how to connect them back together later.
	#in this crude test, 6, 8, and 9 are vertices.: 9 from the borders, 6 and 8 are internal vertices connecting
	#infill edges
	hextile = np.array(  [[0,0,0,0,0,0,6,3,3,6], \
						 [3,0,0,0,0,3,0,0,0,0], \
						 [3,0,0,0,0,3,0,0,0,0], \
						 [0,6,3,3,6,0,0,0,0,0], \
						 [0,8,4,4,8,0,0,0,0,0], \
						 [4,0,0,0,0,4,0,0,0,0], \
						 [4,0,0,0,0,4,0,0,0,0], \
						 [0,0,0,0,0,0,8,4,4,8]],dtype=np.uint8 );	

	hextile3 = np.array(  [[0,0,0,0,0,0,1,1,1,1], \
						 [1,0,0,0,0,1,0,0,0,0], \
						 [1,0,0,0,0,1,0,0,0,0], \
						 [0,1,1,1,1,0,0,0,0,0], \
						 [0,1,1,1,1,0,0,0,0,0], \
						 [1,0,0,0,0,1,0,0,0,0], \
						 [1,0,0,0,0,1,0,0,0,0], \
						 [0,0,0,0,0,0,1,1,1,1]],dtype=np.uint8 );							 
						 
	hextile2 = np.array(  [[5,5,0,0,0,0,0,0,5,5], \
						 [0,0,7,0,0,0,0,8,0,0], \
						 [0,0,7,0,0,0,0,8,0,0], \
						 [0,0,0,7,9,9,8,0,0,0], \
						 [0,0,0,4,3,3,3,0,0,0], \
						 [0,0,4,0,0,0,0,3,0,0], \
						 [0,0,4,0,0,0,0,3,0,0], \
						 [6,6,0,0,0,0,0,0,6,6]],dtype=np.uint8 );	
						 
	linear = np.array([[0,0,0,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,0,0,0,0,0]] );	
						 
	weird = np.array([[0,0,0,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,5,0,0,0,0], \
						 [0,0,5,5,5,5,0,0,0,0], \
						 [0,0,5,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,5,0,0,0,0], \
						 [0,0,0,0,0,0,5,0,0,0]] );						 
	pm2 = pixmapFromFace2(faceFromWires(makeHeartWire(),[] ));
	#pm2 = pixmapFromFaceTriangulation(f,hextile3);
	#pm2 = pixmapFromFace2(f); #make pixmap without inner fill
	#(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([f]);
	#print "Making Edges..."
	#he = makeHexEdges(xMin,yMin,xMax,yMax,zMax);
	#print "Trimming Edges..."
	
	q = time.clock();
	#g = testWirePixmap(f);
	print "Trim Edges: Elapsed %0.3f" % ( time.clock() - q );
	print "Displaying..."
	#print g.edges();
	#displayGraph(g);
	
	pm2.saveImage("c:\\temp\\heartborder.jpg");
	#np.savetxt("c:\\temp\\heartborder.txt",pm2.p,fmt="%d");

	print "Elapsed %0.3f" % ( time.clock() - q );
	
	
	TestDisplay.display.run();
	