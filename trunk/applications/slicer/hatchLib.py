"""
	A Lightweight Hatching Library

"""
import os
import sys
import os.path
import wx
import logging
import time
import traceback
import gc

import math
from OCC  import gp
from OCC  import BRepAdaptor
from OCC import BRepBuilderAPI
from OCC import Extrema
from OCC import Approx
from OCC import GeomAbs
from OCC import BRepExtrema
from OCC import Geom
from OCC import GeomAPI
from OCC import Geom2dAPI
from OCC import BRepBndLib 
from OCC import Bnd;
from OCC import BRep;
from OCC import TopTools
from OCC import TopoDS
from OCC import TopAbs
from OCC import BRepTools
import Wrappers
import TestDisplay
import hexagonlib
import edgegraph3 as eg
import cProfile
import pstats
from OCC.Display.SimpleGui import *


#20% speed boost from psyco
#import psyco
#psyco.full()

log = logging.getLogger('hatchLib');

ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();

display, start_display, add_menu, add_function_to_menu = init_display()

"""
	convienient way to iterate in pairs with loop around to the start
	given a list, return it in pairs, with the last element wrapping around to the first
	if wrap is false, the last element is not returned.
	
	
	Example:
		lst = [1,2,3];
		
		ntuples(lst,2,True)
			(1,2)
			(2,3)
			(3,1)
		
		ntuples(lst,2,False)
			(1,2)
			(2,3)
		
		ntuples(lst,3,True)
			(1,2,3)
			(2,3,1)
			
		ntuples(lst,3,False)
			(1,2,3)
"""
def ntuples(lst, n,wrap=True):
	B = 0;
	if not wrap:
		B = 1;
	return zip(*[lst[i:]+lst[:(i-B)] for i in range(n)])

	
"""
	A faster way to compute things, since we know we are dealing with a flat face:
	get the curves from the face into pcurves, which are 2d curves in the parametric space on the face:
	
		The wire W is a set of edges Ei (i=1,2...nE). 
		1. Explore it onto edges Ei. Use the class TopoDS_Iterator.
		2. Check whether the edge Ei has p-curve on surface. Use the method:

		Handle(Geom2d_Curve) BRep_Tool::CurveOnSurface(const TopoDS_Edge& E, 
								   const TopoDS_Face& F,
								  Standard_Real& First,
								  Standard_Real& Last)

	then, you have a Geom2d_Curve

	make another with this:
		Geom2d_Line (const gp_Pnt2d &P, const gp_Dir2d &V)
			Constructs a line passing through point P and parallel to 
		vector V (P and V are, respectively, the origin 
		and the unit vector of the positioning axis of the line). 	
		
	then, compute intersections with this:
		Geom2dAPI_InterCurveCurve (const Handle(Geom2d_Curve)&C1, const Handle(Geom2d_Curve)&C2, const Standard_Real Tol=1.0e-6)
			Creates an object and computes the 
		intersections between the curves C1 and C2. 	
	
"""
class Hatcher:
	"class that accepts a shape and produces a set of edges from it"
	"usage: Hatcher(...) then hatch() then edges()"
	def __init__(self,wireList,zLevel,bounds):
	
		#self.boundaryWires = Wrappers.makeWiresFromOffsetShape(shape);
		self.boundaryWires = wireList;
		
		self.graph = eg.EdgeGraph();
		#maps the wires into the node graph.
		for w in wireList:
			self.graph.addWire(w,'BOUND');
			
		self.bounds = bounds;
		self.zLevel = zLevel;
		self.HATCH_PADDING = 1; #TODO, should be based on unit of measure
	
	def  hatch(self):
		"""take the a slice and compute hatches
		
			computing the intersections of wires is very expensive, 
			so it is important to do this as efficently as possible.
			
			Inputs: a set of boundary wires that define a face,
			        a set of filling wires that cover a bounding box around the face.
			
			Outputs: 
				trim covering wires/edges by the boundaries, 
				trim boundaries by intersections with wires
				
			optimizations so far:
				* use active wire table to detect when to stop computing intersections with wires.
				 each boundary must be activated before computation stops, and a boundary is finished
				 after intersections have been found, and then stop occurring.
				 
			More optimization can be used by Bnd_BoundSortBox-- which would compare all of the bounding
			boxes at once in c++, but with more glue code.
			
		"""
		log.info("Hatching...");
		q = time.clock();
		hatchWires = self._makeHatchLines22();
		numCompares = 0;
		
		print "Time to Make hatch Lines: %0.3f" % ( time.clock() - q )
		activeBoundaries = {};
		#bndBoxes = {}
		for b in self.boundaryWires:
			#bl = BRepBndLib.BRepBndLib();
			#bbox = Bnd.Bnd_Box();
			#bl.Add(b,bbox);
			#bndBoxes[b] = bbox;
			activeBoundaries[b] = 0;
		print "There are %d boundary wires, %d hatch wires" % ( len(self.boundaryWires),len(hatchWires)  )
		#intersect each line with each boundary				
		for hatchLine in hatchWires:
			
			#b = BRepBndLib.BRepBndLib();
			#wireBounds = Bnd.Bnd_Box();
			#b.Add(hatchLine,wireBounds);

			if len(activeBoundaries) == 0:
				break; #finished hatching, no more active boundaries.

			interSections = [];	#list of intersections for just this single hatch line	
			#display.DisplayShape(hatchLine);
			for boundary in activeBoundaries.keys():
				
				#do not compute if bounding boxes are outside
				#if wireBounds.IsOut(bndBoxes[boundary]):
				#	#print "skipping this comparison"
				#	continue;
					
				brp = BRepExtrema.BRepExtrema_DistShapeShape();
				brp.LoadS1(boundary);
				brp.LoadS2(hatchLine );
				numCompares+= 1;
				if brp.Perform() and brp.Value() < 0.001:
					#print "intersection found!"
					#TODO need to handle the somewhat unusual cases that the intersection is
					#on a vertex
					for k in range(1,brp.NbSolution()+1):
						activeBoundaries[boundary] = 1;
						#try:
						#make the node
						#quite complex depending on where exactly the intersection is.
						if brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsOnEdge:
							#well this sux-- have to check to ensure that the edge is not a local
							#minimum or maximum also.
							if Wrappers.isEdgeLocalMinimum(Wrappers.cast(brp.SupportOnShape1(k)),brp.ParOnEdgeS1(k),brp.PointOnShape1(k)):
								#print "Warning: edge appears to be a local min/max.  Is it a tangent?"
								continue;
							else:
								self.graph.divideEdge(Wrappers.cast(brp.SupportOnShape1(k)),brp.ParOnEdgeS1(k));
						
						if brp.SupportTypeShape2(k) == BRepExtrema.BRepExtrema_IsOnEdge:
							

							
							if brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsVertex:
								#the intersection is on a vertex of the boundary.
								vertex = Wrappers.cast(brp.SupportOnShape1(k));
								if Wrappers.isLocalMinimum(boundary, vertex):
									print "vertex encountered that is a local minimum, skipping it"	
									continue;
							
							#otherwise, vertex was not a local minimum, or was on an edge
							poe = eg.PointOnAnEdge(Wrappers.cast(brp.SupportOnShape2(k)),brp.ParOnEdgeS2(k),brp.PointOnShape2(k));
							interSections.append(poe);
							
						elif brp.SupportTypeShape2(k) == BRepExtrema.BRepExtrema_IsVertex:
							#how on earth to handle this one?
							#this actually means that a vertex can also have an infill node attached to it!
							#for right now let's just ignore it.
							print "WARNING: intersection on vertex of hatch line!"
							pass;
						else:
							raise ValueError("I dont know how to handle this kind of intersection");

					
				else:
					if activeBoundaries[boundary] == 1:
						#finished with this wire. 
						#print "Finished with wire %d" % boundary.__hash__();
						del activeBoundaries[boundary];
					

			#at this point we have all the intersections for this hatch line.
			#add the edges 'inside' the shape to the graph
			#print "Splitting wire by %d intersection points" % len(interSections )
			edgesInside = [];
			edgesInside = eg.splitWire(hatchLine,interSections);
			
			#print "Split returned %d edges" % len(edgesInside);
			# TODO: a speedup here is possible. why add all edges to the graph when only the
			#first and last in the set will do?
			
			#for e in edgesInside:
			#	#fdisplay.DisplayShape(e);
			#	self.graph.addEdge(e,'FILL');
			if len(edgesInside) > 1:
				self.graph.addEdgeListAsSingleEdge(edgesInside,'FILL');
			if len(edgesInside) == 1:
				self.graph.addEdge(edgesInside[0],'FILL');
			
			#test to see if we can break out of the loop.
			#we can stop if we've hit each boundary at least once
			#if len(interSections) == 0 and (len(boundariesFound) ==  len(self.boundaryWires)):
			#	continueHatching = False;
			
			if len(interSections) % 2 == 1:
				log.warn( "Detected Odd Number of intersection points. This is ignored for now.");
				continue;
			
			log.debug("found %d intersection Nodes." % len(interSections));


		log.info("Finished Hatching.");
		print "%d Total Intersections computed." % ( numCompares )
		
		
	
	def _makeHatchLines22(self):
		"make hatch lines using hexlib"
		hex = hexagonlib.Hexagon(0.250,0.01);
		
		xMin = self.bounds[0] - ( self.HATCH_PADDING);
		yMin = self.bounds[1] - ( self.HATCH_PADDING);		 
		xMax = self.bounds[2] + (self.HATCH_PADDING);
		yMax = self.bounds[3] + (self.HATCH_PADDING) ;				
		wires = hex.makeHexForBoundingBox(( xMin,yMin,self.zLevel),(xMax,yMax,self.zLevel ));

		return wires;	
	
	def _makeHatchLines(self):
		"make straight hatch lines."
		xMin = self.bounds[0] - ( self.HATCH_PADDING);
		yMin = self.bounds[1] - ( self.HATCH_PADDING);		 
		xMax = self.bounds[2] + (self.HATCH_PADDING);
		yMax = self.bounds[3] + (self.HATCH_PADDING) ;
		wires = [];
		for y in Wrappers.frange6(yMin,yMax,0.05):
			e = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(xMin,y,self.zLevel),gp.gp_Pnt(xMax,y,self.zLevel));
			#display.DisplayShape(e);
			wires.append(Wrappers.wireFromEdges([e]));
		return wires;


	
	
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
	
def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
	p.print_stats(level);	

			
	
def displayEdgeFromGraph(ed):
	if ed.has_key('node'):
		#these are partial segments of edges"
		e = ed['node'].newEdge();
		#display.DisplayShape( TestDisplay.makeEdgeIndicator(e) );
		display.DisplayShape(e,update=False );	
	if ed.has_key('edgeList'):
		#print "Found EdgeList"
		#these are full edges
		el = ed['edgeList'];
		for e in el:
			display.DisplayShape(e,update=False );
				
def displayAllEdgesAndVertices(h):
	i=0;
	for en in h.graph.allEdgesRandomOrder():
		i+=1;
		displayEdgeFromGraph(en)	
	print "%d edges total" % i

	for n in h.graph.allNodesRandomOrder():
		p = gp.gp_Pnt(n[0],n[1],0);
		display.DisplayShape(Wrappers.make_vertex(p),update=False);

if __name__=='__main__':

	###Logging Configuration
	logging.basicConfig(level=logging.WARN,
						format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
						stream=sys.stdout)	
						
	print "******************************"
	print " HatchLib Unit Tests...."
	print "******************************"
	#w = TestDisplay.makeSquareWire(); #box around 0,0 <--> 5,5 
	#w2=TestDisplay.makeCircleWire(); #circle centered at 2,2, radius 1
	#h = Hatcher([w,w2],0.0,( 0,0,5.0,5.0) );
	gc.disable();
	w = makeHeartWire();
	w2=makeCircleWire();
	h = Hatcher([w,w2],0.0,( -6,0,10.0,10.0) );
	
	#display.DisplayShape(w);
	#display.DisplayShape(w2);
		
	#runProfiled('h.hatch()',0.9);
	t = Wrappers.Timer();
	h.hatch();
	#h.hatch2();
	print "Hatching Finished",t.finishedString();
	print "There are %d fill edges" % len(h.graph.fillEdges)
	
	#display all edges in the graph
	displayAllEdgesAndVertices(h)
	
	#display edges in walk-to-fill order
	#each entry is a list of nodes that form a path
	print h.graph.walkEdges();
	#for en in he.graph.walkEdges():
	#	for p in Wrappers.pairwise(en):
	#		#each pair is a set of nodes that form an edge
	#		displayEdgeFromGraph( h.graph.getEdge(p[0],p[1]) );
			

	print "Done.";
	display.FitAll();
	start_display()

	