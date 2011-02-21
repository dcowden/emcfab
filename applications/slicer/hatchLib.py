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
from OCC import BRepTools
import Wrappers
import TestDisplay
import hexagonlib
import edgegraph3 as eg
import cProfile
import pstats

log = logging.getLogger('hatchLib');

ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();

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
		"take the a slice and compute hatches"
		log.info("Hatching...");
		
		hatchWires = self._makeHatchLines();
		#for h in hatchWires:
		#	TestDisplay.display.showShape(h);
		
		#log.warn( "Created %d Hatch Wires" % len(hatchWires));
		boundariesFound = {};
		#intersect each line with each boundary
		continueHatching = True;
		
		for hatchLine in hatchWires:
			if not continueHatching:
				break;
			else:
				log.debug( "Moving to next Hatch Line");

			interSections = [];	#list of intersections for just this single hatch line	
			#TestDisplay.display.showShape(hatchLine);
			for boundary in self.boundaryWires:
				brp = BRepExtrema.BRepExtrema_DistShapeShape();
				brp.LoadS2(boundary);
				brp.LoadS1(hatchLine );
				
				if brp.Perform() and brp.Value() < 0.001:
					#print "intersection found!"
					#TODO need to handle the somewhat unusual cases that the intersection is
					#on a vertex
					for k in range(1,brp.NbSolution()+1):
						boundariesFound[boundary] = 1;
						#try:
						#make the node
						if brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsOnEdge:
							poe = eg.PointOnAnEdge(Wrappers.cast(brp.SupportOnShape1(k)),brp.ParOnEdgeS1(k),brp.PointOnShape1(k));
							interSections.append(poe);
							
							#also divide the edge in the graph as this point
							self.graph.divideEdge(Wrappers.cast(brp.SupportOnShape2(k)),brp.ParOnEdgeS2(k));
							
						elif brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsVertex:
							#how on earth to handle this one?
							#this actually means that a vertex can also have an infill node attached to it!
							#for right now let's just ignore it.
							pass;
						else:
							raise ValueError("I dont know how to handle this kind of intersection");
						#except:
						#	log.warn("Problem Creating an intersection node... Ignoring.");
					
				else:
					log.debug( "No Intersections Found");

			#at this point we have all the intersections for this hatch line.
			#add the edges 'inside' the shape to the graph
			#print "Splitting wire by %d intersection points" % len(interSections )
			edgesInside = eg.splitWire(hatchLine,interSections);
			#print "Split returned %d edges" % len(edgesInside);
			
			for e in edgesInside:
				self.graph.addEdge(e,'FILL');
			
			#test to see if we can break out of the loop.
			#we can stop if we've hit each boundary at least once
			if len(interSections) == 0 and (len(boundariesFound) ==  len(self.boundaryWires)):
				continueHatching = False;
			
			if len(interSections) % 2 == 1:
				log.warn( "Detected Odd Number of intersection points. This is ignored for now.");
				continue;
			
			log.debug("found %d intersection Nodes." % len(interSections));


		log.info("Finished Hatching.");

	
	def _makeHatchLines(self):
		"make hatch lines using hexlib"
		hex = hexagonlib.Hexagon(0.25,0.05);
		
		xMin = self.bounds[0] - ( self.HATCH_PADDING);
		yMin = self.bounds[1] - ( self.HATCH_PADDING);		 
		xMax = self.bounds[2] + (self.HATCH_PADDING);
		yMax = self.bounds[3] + (self.HATCH_PADDING) ;		
		
		wires = hex.makeHexForBoundingBox(( xMin,yMin,self.zLevel),(xMax,yMax,self.zLevel ));
		print "made %d hex wires." % ( len(wires) );
		#edges = [];
		#allEdges = TopTools.TopTools_HSequenceOfShape();
		#for w in wires:
		#	ww = Wrappers.Wire(w);
		#	for e in ww.edges():
		#		allEdges.Append(e);

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

	
def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
	p.print_stats(level);	
	
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

	w = makeHeartWire();
	h = Hatcher([w],0.0,( -6,0,6.0,6.0) );
	
	#TestDisplay.display.showShape(w);
	#TestDisplay.display.showShape(w2);
		
	#runProfiled('h.hatch()',0.9);
	t = Wrappers.Timer();
	h.hatch();
	print "Hatching Finished",t.finishedString();
	#display all edges in the graph
	for en in h.graph.allEdgesRandomOrder():
		e = en.newEdge();
		#TestDisplay.display.showShape( TestDisplay.makeEdgeIndicator(e) );
		TestDisplay.display.showShape(e );	

	

	print "Done.";
	TestDisplay.display.run();

	