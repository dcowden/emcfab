"""
	A Lightweight Hatching Library

"""
import os
import sys
import os.path
import time

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

import edgegraph as eg
import TestObjects
import Util
import Wrappers
import OCCUtil

#20% speed boost from psyco
#import psyco
#psyco.full()


ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();

	

class Hatcher:
	"class that accepts a shape and produces a set of edges from it"
	"usage: Hatcher(...) then hatch() then edges()"
	def __init__(self,wireList,zLevel,bounds,spacing,fillAngle):
	
		self.fillAngle = fillAngle; #TODO: need to actually use this
		self.boundaryWires = wireList;
		self.spacing = spacing;	
		self.bounds = bounds;
		self.zLevel = zLevel;
		self.HATCH_PADDING = 5.0; #TODO, should be based on unit of measure
	
		self.graphBuilder = eg.EdgeGraphBuilder();
	
	def wires(self):
		"""
			return a list of wires that resulted from the hatching operation.
			TODO, right now there is a mess, need to return this 
		
		"""
		return [];
	
	
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
		q = time.clock();
		hatchWires = self._makeHatchLines();
		numCompares = 0;
		bbuilder = BRep.BRep_Builder();
		comp = TopoDS.TopoDS_Compound();
		bbuilder.MakeCompound(comp);
		
		#print "Time to Make hatch Lines: %0.3f" % ( time.clock() - q )
		
		for b in self.boundaryWires:
			bbuilder.Add(comp,b);
			self.graphBuilder.addBoundaryWire(b);
			
		#print "There are %d boundary wires, %d hatch wires" % ( len(self.boundaryWires),len(hatchWires)  )
		#intersect each line with each boundary				
		
		brp = BRepExtrema.BRepExtrema_DistShapeShape();
		brp.LoadS1(comp);
		
		for hatchLine in hatchWires:

			interSections = [];	#list of intersections for just this single hatch line
			closePoints = []; #extrema that are not intersectionsf or this single hatch line
			
			brp.LoadS2(hatchLine );
			numCompares+= 1;
			result = brp.Perform();
			
			"""
				tricky thing: for a given hatch line ( whether a line or a string of hexes ),
				we want to check for both intersections and close points that would result in overdrawing.
				if the line has zero intersections, then it is not in play, and should not be considered.
				if the line has at least one intersection, then it is in play, and extrema that are not intersections should
				also be included for later processing.
			"""
			if result and brp.Value() < 0.050: #if < tolerance we have an intersection. if < filament width we have a close sitation
				print "intersection found!"
				#TODO need to handle the somewhat unusual cases that the intersection is
				#on a vertex
				for k in range(1,brp.NbSolution()+1):
					if brp.Value() < 0.001:	#there is at least one intersection on this wire. there may also be extrema
						print "spot on match" 					
						#try:
						#make the node
						#quite complex depending on where exactly the intersection is.
						if brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsOnEdge:
							#well this sux-- have to check to ensure that the edge is not a local
							#minimum or maximum also.
							if OCCUtil.isEdgeLocalMinimum(OCCUtil.cast(brp.SupportOnShape1(k)),brp.ParOnEdgeS1(k),brp.PointOnShape1(k)):
								print "Warning: edge appears to be a local min/max.  Is it a tangent?"
								continue;
							else:
								#self.boundaryIntersectionsByEdge = {} #boundary intersections, hashed by edges
								self.graphBuilder.addPointOnBoundaryEdge(OCCUtil.cast(brp.SupportOnShape1(k)),brp.ParOnEdgeS1(k),brp.PointOnShape1(k));

						if brp.SupportTypeShape2(k) == BRepExtrema.BRepExtrema_IsOnEdge:
							if brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsVertex:
								#the intersection is on a vertex of the boundary.
								vertex = OCCUtil.cast(brp.SupportOnShape1(k));
							
							#otherwise, vertex was not a local minimum, or was on an edge
							poe = eg.PointOnAnEdge(OCCUtil.cast(brp.SupportOnShape2(k)),brp.ParOnEdgeS2(k),brp.PointOnShape2(k));
							interSections.append(poe);
							
						elif brp.SupportTypeShape2(k) == BRepExtrema.BRepExtrema_IsVertex:
							#how on earth to handle this one?
							#this actually means that a vertex can also have an infill node attached to it!
							#for right now let's just ignore it.
							print "WARNING: intersection on vertex of hatch line!"
							pass;
						else:
							raise ValueError("I dont know how to handle this kind of intersection");
					else: #brp.Value is between 0.001 and 0.05
						print "intersection is close";
						#we know this is a place where the boundary is close to a fill contour.
						#our goal is to eventually remove it from the list. Support1 is the boundary.
						#print "found extremum close but not intersecting, distance = %0.3f" %  ( brp.Value() )
						if brp.SupportTypeShape1(k) == BRepExtrema.BRepExtrema_IsOnEdge:
							poeBound = eg.PointOnAnEdge(brp.SupportOnShape1(k), brp.ParOnEdgeS1(k),brp.PointOnShape1(k));
							closePoints.append((poeBound,'EDGE',brp.SupportOnShape2(k)));								
						elif brp.SupportTypeShape2(k) == BRepExtrema.BRepExtrema_IsVertex:
							#here a vertex is closest to a fill line.
							poeBound = eg.PointOnAnEdge(brp.SupportOnShape1(k), 0,brp.PointOnShape1(k));
							closePoints.append((poeBound,'VERTEX',brp.SupportOnShape2(k)));		
				
			if len(interSections) % 2 == 1:
				print "Detected Odd Number of intersection points. This is ignored for now.";
				continue;
			
			if len(interSections) == 0:
				print "Hatch has no intersections-- discarding";
				continue;
						
			#at this point we have all the intersections for this hatch line.
			#we also know we have at least one intersection.
			
			if len(closePoints) > 0:
				#there were extrema ( points where this hatch is close to a boundary but doesnt intersect.
				#we do this here instead of inline because we have to avoid hatch lines that are close, but do not actually
				#intersect a wire. ( ie, they are 'just outside' of the boundary )
				for cp in closePoints:
					self.graphBuilder.addPointsTooClose(cp[0], cp[1]);
					#display.DisplayShape(cp[0].edge );
					#display.DisplayShape(cp[2] );
					
			#add the edges 'inside' the shape to the graph
			#print "Splitting wire by %d intersection points" % len(interSections )
			edgesInside = [];
			edgesInside = eg.splitWire(hatchLine,interSections);
	
			#returned value is a list of lists. each entry is a chain of edges
			for e in edgesInside:
				self.graphBuilder.addFillEdges(e);
							
			#test to see if we can break out of the loop.
			#we can stop if we've hit each boundary at least once
			#if len(interSections) == 0 and (len(boundariesFound) ==  len(self.boundaryWires)):
			#	continueHatching = False;
			
		print "%d Total Intersections computed." % ( numCompares )
		self.graphBuilder.buildGraph();
		
	"""
		Returns a list of wires, that will fill the face supplied at creation.
		There can be several wires since it may not be possible to fill the entire area
		with a single path.
	"""
	def getWires(self):
		q = time.clock();
		en = self.graphBuilder.walkEdges();
		
		#print "Walked Edges-- %d total paths, %0.3f sec" % (len(en), ( time.clock() - q ) );
		wires = [];
		for path in en:
			wb = OCCUtil.WireBuilder();
			for edge in Util.pairwise(path):
				#each pair is a set of nodes that form an edge in the graph. We need to ge the OCC edges from each
				for occEdge in self.graphBuilder.getEdges(edge[0],edge[1]):
					wb.add( OCCUtil.cast(occEdge) );
				wires.append(wb.wire() );
		return wires;
		
	def _makeHexHatchLines(self):
		"make hatch lines using hexlib"
		hex = hexagonlib.Hexagon(0.5,.02);
		
		xMin = self.bounds[0] - ( self.HATCH_PADDING);
		yMin = self.bounds[1] - ( self.HATCH_PADDING);		 
		xMax = self.bounds[2] + (self.HATCH_PADDING);
		yMax = self.bounds[3] + (self.HATCH_PADDING) ;				
		wires = hex.makeHexForBoundingBox(( xMin,yMin,self.zLevel),(xMax,yMax,self.zLevel ));

		return wires;	
	
	def _makeHatchLines(self):
		"""
			make straight hatch lines.
			TODO: need to use fillAngle to rotate the lines as well. Trsf object 
		"""
		xMin = self.bounds[0] - ( self.HATCH_PADDING);
		yMin = self.bounds[1] - ( self.HATCH_PADDING);		 
		xMax = self.bounds[2] + (self.HATCH_PADDING);
		yMax = self.bounds[3] + (self.HATCH_PADDING) ;
		wires = [];
		for y in Util.frange6(yMin,yMax,self.spacing):
			e = OCCUtil.edgeFromTwoPoints(gp.gp_Pnt(xMin,y,self.zLevel),gp.gp_Pnt(xMax,y,self.zLevel));
			wires.append(OCCUtil.wireFromEdges([e]));
		return wires;
	
def runProfiled(cmd,level=1.0):
	"run a command profiled and output results"
	cProfile.runctx(cmd, globals(), locals(), filename="slicer.prof")
	p = pstats.Stats('slicer.prof')
	p.sort_stats('cum')
	p.print_stats(level);	

			
	
def displayEdgeFromGraph(ed,refreshNow=False):
	#if ed == None:
	#	return;
	if ed.has_key('node'):
		#these are partial segments of edges"
		e = ed['node'].newEdge();
		#display.DisplayShape( TestDisplay.makeEdgeIndicator(e) );
		display.DisplayShape(e,update=refreshNow );
	if ed.has_key('edge'):
		display.DisplayShape(ed['edge'],update=refreshNow);
		
	if ed.has_key('edgeList'):
		#print "Found EdgeList"
		#these are full edges
		for e in ed['edgeList']:
			display.DisplayShape(e,update=refreshNow );
	
	
def debugGraph(g):
	print "There are %d nodes total" % ( len(g.nodes()) )
	for n in g.nodes_iter():
		print "%s, %d neighbors" % ( str(n), len ( g.neighbors(n) ) )
		
def displayAllEdgesAndVertices(g):
	i=0;
	for en in g.edges_iter(data=True):
		i+=1;
		displayEdgeFromGraph(en[2])	
	print "%d edges total" % i
	i=0
	for n in g.nodes_iter():
		i+=1;
		p = gp.gp_Pnt(n[0],n[1],0);
		display.DisplayShape(OCCUtil.make_vertex(p),update=False);
	print "%d nodes total" % i

if __name__=='__main__':

	from OCC.Display.SimpleGui import *
	display, start_display, add_menu, add_function_to_menu = init_display()		
					
	print "******************************"
	print " HatchLib Unit Tests...."
	print "******************************"
	#w = TestDisplay.makeSquareWire(); #box around 0,0 <--> 5,5 
	#w2=TestDisplay.makeCircleWire(); #circle centered at 2,2, radius 1
	#h = Hatcher([w,w2],0.0,( 0,0,5.0,5.0) );

	w = TestObjects.makeHeartWire();
	w2=TestObjects.makeCircleWire2();
	h = Hatcher([w,w2],0.0,( -6,0,10.0,10.0), 0.25, 45 );

	#display.DisplayShape(w);
	#display.DisplayShape(w2);
		
	#runProfiled('h.hatch()',0.9);
	t = Util.Timer();
	h.hatch();
	q = time.clock();
	ww = h.getWires();
	print "got Wires in %0.3f" % ( time.clock() - q );
	display.DisplayShape(ww,False );
	display.FitAll();
	g = h.graphBuilder.graph;
	#debugGraph(g)
	#h.hatch2();
	print "Hatching Finished",t.finishedString();
	#print "There are %d fill edges" % len(h.graph.fillEdges)
	
	if False:
		for e in g.edges_iter(data=True):
			displayEdgeFromGraph(e[2]);

	if False:
		for e in h.graphBuilder.fillEdges:
			edge = g.get_edge_data(e[0],e[1]);
			displayEdgeFromGraph(edge,True);
			#time.sleep(.5)
	#display all edges in the graph
	#displayAllEdgesAndVertices(g)
	
	#display edges in walk-to-fill order
	#each entry is a list of nodes that form a path

	"""
	if True:
		
		q = time.clock();
		en = h.graphBuilder.walkEdges();
		print "Walked Edges-- %d total paths, %0.3f sec" % (len(en), ( time.clock() - q ) );
		for path in en:
				for edge in Util.pairwise(path):
					#each pair is a set of nodes that form an edge
					#print "Display Edge:",edge[0],edge[1]
					displayEdgeFromGraph( g.get_edge_data(edge[0],edge[1]),False);
					#time.sleep(.75)
	"""
	print "Done.";
	display.FitAll();
	start_display()

	