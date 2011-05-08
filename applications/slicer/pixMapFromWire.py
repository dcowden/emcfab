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
from OCC import BRepMesh
from OCC.Utils.Topology import Topo
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
	"""
	PIXEL = 0.02;
	DEFLECTION = PIXEL / 2.0;

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

	#we will draw the outer wire as a filled polygon, then
	#draw the inner wire as another filled polygon
	
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
	
	PIXEL = 0.1 ;
	DEFLECTION = PIXEL / 4.0;

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
	pixmap.drawPolygon(outerPoints,0,1);
	#drawWire(outerPoints,pixmap);
	#pixmap.saveImage("c:\\temp\\heartborder-1.bmp");
	
	#get the other wires.
	wires = []
	wires.append(outerPoints);
	for w in Topo(face).wires():
		if not w.IsSame(ow):
			wp = tuplesFromWire(w,DEFLECTION);
			pixmap.drawPolygon(wp,1,0);
			wires.append(wp);
			

	#pixmap.saveImage("c:\\temp\\heartborder-2.bmp");
	if fillpattern is not None:
		pixmap.tileOnto(fillpattern);
	pixmap.saveImage("c:\\temp\\heartborder-3.bmp");
	
	#now draw the borders, while marking vertices.
	#after this runs, all vertices in the graph should have value 9
	#vertexList = [];
	for w in wires:
		for et in Wrappers.pairwise(w):
			#vertexList.extend(pixmap.drawLine2(et[0],et[1],1));
			pixmap.drawLine(et[0],et[1],2);

	pixmap.saveImage("c:\\temp\\heartborder-4.bmp");
	return pixmap;
	
def pixmapFromFaceTriangulation(face,fillpattern):
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
	
	PIXEL = 0.1;
	DEFLECTION = PIXEL / 10.0;
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
	bb = TopExp.TopExp_Explorer();
	bb.Init(face,TopAbs.TopAbs_WIRE);
	edgeNum = 1;
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
				#pixmap.set(tP(pnt),7);
				edgeNum += 1;
				#plot the line
				#if lastPnt != None: pixmap.drawLine(tP(lastPnt),tP(pnt),edgeNum );				
				lastPnt = pnt;
		bb.Next();	
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
	
if __name__=='__main__':

	w = makeHeartWire();
	w2 = makeCircleWire();

	#pm = pixmapFromWires([w2,w] );
	f = faceFromWires(w,[w2]);
	q = time.clock();
	#pm = pixmapFromFace(f);
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
	pm2 = pixmapFromFace2(f,hextile3);
	#pm2 = pixmapFromFaceTriangulation(f,hextile);
	#pm2 = pixmapFromFace2(f); #make pixmap without inner fill
	#(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([f]);
	#print "Making Edges..."
	#he = makeHexEdges(xMin,yMin,xMax,yMax,zMax);
	#print "Trimming Edges..."
	
	#q = time.clock();
	#g = testWirePixmap(f);
	#print "Trim Edges: Elapsed %0.3f" % ( time.clock() - q );
	#print "Displaying..."
	#print g.edges();
	#displayGraph(g);
	
	pm2.saveImage("c:\\temp\\heartborder.jpg");
	np.savetxt("c:\\temp\\heartborder.txt",pm2.p,fmt="%d");

	print "Elapsed %0.3f" % ( time.clock() - q );
	
	
	TestDisplay.display.run();
	