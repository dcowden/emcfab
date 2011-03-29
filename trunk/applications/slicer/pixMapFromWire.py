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
import pixmaptest
import Wrappers
import TestWires
import TestDisplay
import pixMapTileTest
import breshamtest
import numpy as np

ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();

def tP(pnt):
	return (pnt.X(),pnt.Y());

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
	
def pixmapFromFace(face,fillpattern):
	"""
		create a filled pixmap from a defined face.
		
		ideally, this is really really fast, based on a triangulation of the face.
		a face with 800x600 pixels took 56ms to raster with this algo. @ 0.012" that is a 6x8 " part!

		this creates a pixel map that has all the interior pixels filled
	"""
	
	PIXEL = 0.01;
	DEFLECTION = PIXEL / 8.0;
	triangulateAtDeflection(face,DEFLECTION);
	
	#get bounding box
	(xMin,yMin,zMin,xMax,yMax,zMax) = boundingBox([face]);
	
	#make pixmap
	pixmap = pixmaptest.pixmap((xMin,yMin),(xMax,yMax),PIXEL);
	
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
	pixmap.tileOnto(fillpattern);
	
	#mark boundaries
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
				pixmap.set(tP(pnt),7);
						
				#plot the line
				if lastPnt != None: pixmap.drawLine(tP(lastPnt),tP(pnt),7 );				
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
	hextile = np.array(  [[4,6,0,0,0,0,0,0,6,4], \
						 [0,0,4,0,0,0,0,4,0,0], \
						 [0,0,4,0,0,0,0,4,0,0], \
						 [0,0,0,6,4,4,6,0,0,0], \
						 [0,0,0,6,4,4,6,0,0,0], \
						 [0,0,4,0,0,0,0,4,0,0], \
						 [0,0,4,0,0,0,0,4,0,0], \
						 [4,6,0,0,0,0,0,0,6,4]] );	
	
	linear = np.array([[0,0,0,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,0,0,0,0,0], \
						 [0,0,0,0,0,0,0,0,0,0], \
						 [0,0,0,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,0,0,0,0,0], \
						 [0,0,0,0,0,0,0,0,0,0]] );	
						 
	weird = np.array([[0,0,0,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,5,0,0,0,0], \
						 [0,0,5,5,5,5,0,0,0,0], \
						 [0,0,5,0,0,0,0,0,0,0], \
						 [5,5,5,5,5,5,5,5,5,5], \
						 [0,0,0,0,0,5,0,0,0,0], \
						 [0,0,0,0,0,0,5,0,0,0]] );						 
	pm2 = pixmapFromFace(f,hextile);
	
	pm2.saveImage("c:\\temp\\heartborder.jpg");
	np.savetxt("c:\\temp\\heartborder.txt",pm2.p,fmt="%d");
	print pm2.p.shape;
	print "Elapsed %0.3f" % ( time.clock() - q );
	

	TestDisplay.display.run();
	