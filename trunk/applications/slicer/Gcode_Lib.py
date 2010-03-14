"""
	A library to create Gcode from OCC objects.
	The library is intended to be used to fairly easily produce Gcode toolpaths
	from OCC objects
	
	The library is currently a 3-axis library only.
	It is assumed that the axes of the model correspond to those of the machine.
	
	Copyright [2010] [Dave Cowden ( dave.cowden@gmail.com)]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0	

"""

from OCC import STEPControl,TopoDS, TopExp, TopAbs,BRep,gp,BRepBuilderAPI,BRepTools,BRepAlgo,BRepBndLib,Bnd,StlAPI,BRepAlgoAPI
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
import os
import sys
import os.path
import logging
import time
import math
import svgTemplate
from string import Template

log = logging.getLogger('gcode-lib');
log.setLevel(logging.WARN);					
###
### TODO:
###   consider G90 and G91 ( incremental and absolute modes )
###   consider circles ( arcs )
###   prevent naive redundant linear paths by tracking the last move

"""
	Module Settings

"""
DEFAULT_NUMBER_FORMAT = "%0.2f";
DEFAULT_FEEDRATE = 1.0;
DEFAULT_DEFLECTION=0.001;
OMIT_UNCHANGED_AXES=False;
TOLERANCE=0.00001;

#####
#utility class instances
#available to all methods
#####
#Brep_Tools = BRepTools.BRepTools();
Brep_Tool = BRep.BRep_Tool();
BRepLProp_CurveTool = BRepLProp.BRepLProp_CurveTool();
#TopoDS = TopoDS.TopoDS();
#TopExp = TopExp.TopExp()
#TopExp_Explorer = TopExp.TopExp_Explorer();
ts = TopoDS.TopoDS();

def close(x,y):
	if x == None or y == None:
		return False;
	
	return abs(x - y ) < TOLERANCE;
	
def printPoint(point):
	return "x=%0.4f,y=%0.4f,z=%0.4f" % (point.X(), point.Y(), point.Z());
	
"""
	State machine for creating gcode.
	This class will track the current position, which
	makes it easier to move along a particular path.
	
	Each command returns a list of commands that correspond to
	the desired action, or an empty list if no action is needed
	
	The class also stores the vector of the last motion,
	so that naive gcode can be removed. ( IE, several motions in the same
	direction are combined into a single motion )

"""
class PathGenerator:
	def __init__(self,generator):
		self.generator = generator;
		self.reset();
		self.startUp = "G90 ";
		self.safeHeight = "3";
		self.numberFormat = DEFAULT_NUMBER_FORMAT;
		self.verbose = False;
		self.useArcs = False;
		
	def getResults(self):
		return generator.results();
		
	def debugCurrentPosition(self):
		if self.currentX and self.currentY and self.currentZ:
			print "Current Position: X=%0.4f,Y=%0.4f,Z=%0.4f" % (self.currentX, self.currentY, self.currentZ);
		else:
			print "Current Position is undefined."
		
	def comment(self,cmnt):
		generator.comment(cmnt);
		
	def reset(self):
		
		self.currentX=None;
		self.currentY=None;
		self.currentZ=None;
		self.lastMove = None;
	
	def isPointClose(self,point,tolerance=0.0001):
		if (self.currentX == None) or (self.currentY == None) or (self.currentZ == None):
			return False;
		return (abs(self.currentX - point.X()) < tolerance) and (abs(self.currentY - point.Y()) < tolerance) and (abs(self.currentZ - point.Z()) < tolerance);
		       
	"""
		Follow an edge. A command will be issued ( if necessary ) to move
		to the beginning of the edge, then to the end along the commanded edge.
		
		Usually edges will be chained together so that the end of one is the
		same of the beginning of the next.
	"""
	def followEdge(self,edgeWrapper,feed):
		#print str(edgeWrapper);
		if self.verbose:
			generator.comment("Follow: " + str(edgeWrapper));
		
		#check to see if we should reverse this edge. Perhaps reversing it will work better.
		if not self.isPointClose(edgeWrapper.firstPoint,0.005):
			#print "moving to start."
			generator.moveTo(edgeWrapper.firstPoint,feed,"G00");		
		
		if edgeWrapper.isLine():
			generator.lineTo(edgeWrapper.lastPoint,feed);
			
		elif edgeWrapper.isCircle() and useArcs:
			circle = edgeWrapper.curve.Circle();
			center = circle.Location();
			
			axisDir = circle.Axis().Direction(); #need this to determine which way the arc goes			
			#assume we are essentially in 2d space, so we're looking only at wehter the
			#axis of the circle is +z or -z
			zDir = gp.gp().DZ();
			
			if edgeWrapper.reversed:
				zDir = zDir.Reversed();
						
			#TODO: handle incremental coordinates
			generator.arc(center.X()-edgeWrapper.firstPoint.X(),center.Y()-edgeWrapper.firstPoint.Y(),
				edgeWrapper.lastPoint.X(),edgeWrapper.lastPoint.Y(),
				edgeWrapper.lastPoint.Z(),zDir.IsEqual(axisDir,TOLERANCE));
			self.lastMove = None;
			
		else:
			edge = edgeWrapper.edge;
			range = Brep_Tool.Range(edge);
			log.debug( "Edge Bounds:" + str(range) );
			hc= Brep_Tool.Curve(edge);
			ad = GeomAdaptor.GeomAdaptor_Curve(hc[0]);
			log.debug(  "Edge is a curve of type:" + str(ad.GetType()));
			
			p1 = hc[1];
			p2 = hc[2];
			gc = GCPnts.GCPnts_QuasiUniformDeflection(ad,0.001,p1,p2);
			i=1;
			numPts = gc.NbPoints();
			log.debug( "Discretized Curve has" + str(numPts) + " points." );
			while i<=numPts:
				if edge.Orientation() == TopAbs.TopAbs_FORWARD:
					tPt = gc.Value(i);
				else:
					tPt = gc.Value(numPts-i+1);
				i+=1;
				generator.movePt(tPt,feed);
			
			#last point is the end
			generator.lineTo(edgeWrapper.lastPoint);
		
	#follow the segments in a wire
	#wire is a WireWrapper object
	def followWire(self,wireWrapper, feed ):
		#log.debug( "Following Wire:" + str(wireWrapper));
		#bwe = BRepTools.BRepTools_WireExplorer(wireWrapper.wire);
		wr = wireWrapper.getSortedWire() ;
		for ew in wr.edgeList:
			self.followEdge(ew,feed );
		
	def removeLastNonCommentMove(self):
		"removes statements up to the last non-comment move. Used for naive move tracking"
		removedCommand = False;
		while not removedCommand and len(self.cmdBuffer) > 0:
			a = self.cmdBuffer.pop();
			if not a.startswith('('):
				return;
		
	"""
		return the gcode required to move to the provided point.
		If no gcode is required, "" is returned instead
	
	"""
	def move(self, x = None, y = None, z = None,  feed=None, cmd="G01"):
		
		
		if x == None: x = self.currentX
		if y == None: y = self.currentY
		if z == None: z = self.currentZ
		if feed == None: feed = self.currentFeed;
		
		#rapids reset naive move tracking
		if cmd != 'G01':
			self.lastMove = None;
	
		#if there is no move at all, return immediately
		if ( close(x , self.currentX ) and close(y ,self.currentY)  and close(z , self.currentZ) ):
			log.debug(  "No move required" );
			return "";

		#compute direction of the move, to filter out naive moves
		if  self.currentX != None and self.currentY != None and self.currentZ != None and self.currentFeed != None:
			#ok we are doing a move, and we know what our current location and feed is
			#so we can compute a direction
			currentPoint = gp.gp_Pnt(self.currentX, self.currentY, self.currentZ);
			newPoint = gp.gp_Pnt(x,y,z);
			proposedMove = gp.gp_Vec(currentPoint,newPoint);
			
			if self.lastMove:
				#we have a last move. compare this one to see if they are the same direction
				if ( proposedMove.IsParallel(self.lastMove,TOLERANCE )) and ( self.currentFeed == feed) :
					#caught a naive move
					log.debug("Caught a naive move. Adjusting to remove the extra");
					#TODO this approach only works with absolute coordinates
					#in incremental coordinates, we have to adjust the new vector to move the same
					#as all of the previous moves!
					self.removeLastNonCommentMove();
					#self.comment("Removed Move");
					#remove last non-comment move
					
			#store this one as the last move
			self.lastMove = proposedMove;
		
		#compare the direction of this move to the previous move.
		cmds=[]
		cmds.append(cmd);
		if not close(x, self.currentX):
			cmds.append( ("X" + self.numberFormat)  % (x) );
			self.currentX = x
		if not close(y , self.currentY):
			cmds.append( ("Y" + self.numberFormat)  % (y) );
			self.currentY = y
		if not close( z, self.currentZ):
			cmds.append( ("Z" + self.numberFormat)  % (z) );
			self.currentZ = z
		if feed != self.currentFeed:
			cmds.append(("F" + self.numberFormat) % (feed) );
			self.currentFeed = feed;	
			
		self.addCommand( " ".join(cmds));
		#self.debugCurrentPosition();

"makes gcode toolpaths"
class GCodePathGenerator:
	def __init__(self,feedRate,zLevel):
		self.LINE = "X%0.4f Y%0.4f Z%0.4f F%0.4f ";
		self.ARC = "I%0.4f J%04.F X%0.4f Y%0.4f Z%0.4f F%0.4f ";
		self.paths = [];	
		self.feedRate = feedRate;

	def startLayer(self,zLevel):
		return;
	
	def endLayer(self):
		return;
		
	def startObject(self,sliceSet):
		self.paths.append("G90");
		
	def endObject(self):
		self.paths.append( "M2");
		
	def comment(self,msg):
		self.paths.append("(" + msg + ")");
		
	def moveTo(self,x,y,z):
		self.paths.append("G00 " + self.LINE %( x, y,z,self.feedRate ));
	
	def lineTo(self,x,y,z):
		self.paths.append("G01 " + self.LINE %( x, y,z,self.feedRate ));
	
	def arcTo(self,centerX,centerY, x,y,z,ccw ):
		if ccw:
			c = "G02 ";
		else:
			c = "G03 ";
		self.paths.append(c +self.ARC % ( centerX,centerY,x,y,z,self.feedRate) );
	def results(self):
		return self.paths.join("\n");			
		
		
"makes an svg file containing the layers "
class SVGPathGenerator:
	def __init__(self):
		#initialize stuff here
		self.pointFormat = "%0.4f %0.4f ";
		self.paths = [];
		
	def startObject(self,sliceSet):
		self.sliceSet = sliceSet
		self.title="Untitled";
		self.description="No Description"
		self.unitScale = 3.7;
		self.units = sliceSet.analyzer.guessUnitOfMeasure();
		self.NUMBERFORMAT = "%0.4f";
		self.layers = [];
		
	def comment(self,msg):
		return;
	
	def startLayer(self,slice):
		layer = SVGLayer(slice,self.unitScale ,self.NUMBERFORMAT);
		self.layers.append(layer);
		self.currentLayer = layer;
		
	def endLayer(self);
		return;
		
	def moveTo(self,x,y,z):
		self.currentLayer.paths.append("M " + self.pointFormat %( x, y) );
	
	def lineTo(self,x,y,z):
		self.currentLayer.paths.append("L " + self.pointFormat % (x, y) );
	
	def arcTo(self,centerX,centerY, x,y,z,ccw ):
		return;
		
	def endObject(self):
		return;
		
	def results(self):
		
		top = Template(svgTemplate.topSection);
		layer = Template(svgTemplate.pathSection);
		bottom = Template(svgTemplate.bottomSection);
		
		#build the top and bottom portion of the document
		p = {}:
		p['unitScale'] = self.unitScale;
		p['units'] = self.units;
		p['description'] = self.description;
		p['title']=self.title;
		p['sliceHeight'] = self.NUMBERFORMAT % self.sliceSet.sliceHeight;
		p['xMin'] = self.NUMBERFORMAT % self.sliceSet.analyzer.xMin;
		p['xMax'] = self.NUMBERFORMAT % self.sliceSet.analyzer.xMax;
		p['xRange'] = self.NUMBERFORMAT % self.sliceSet.analyzer.xDim;
		p['yMin'] = self.NUMBERFORMAT % self.sliceSet.analyzer.yMin;
		p['yMax'] = self.NUMBERFORMAT % self.sliceSet.analyzer.yMax;
		p['yRange'] = self.NUMBERFORMAT % self.sliceSet.analyzer.yDim;
		p['zMin'] = self.NUMBERFORMAT % self.sliceSet.analyzer.zMin;
		p['zMax'] =	self.NUMBERFORMAT % self.sliceSet.analyzer.zMax;	
		p['zRange'] = self.NUMBERFORMAT % self.sliceSet.analyzer.zDim;
		
		#svg specific properties
		p['xTranslate']=(-1)*self.sliceSet.analyzer.xMin
		p['yTranslate']=(-1)*self.sliceSet.analyzer.yMin
		
		#put layer dims as nicely formatted numbers
		p['xMinText'] = self.NUMBERFORMAT % self.sliceSet.analyzer.xMin; 
		p['xMaxText'] = "self.NUMBERFORMAT % self.sliceSet.analyzer.xMax;
		p['yMinText'] = self.NUMBERFORMAT % self.sliceSet.analyzer.yMin; 
		p['yMaxText'] = self.NUMBERFORMAT % self.sliceSet.analyzer.yMax;
		p['zMinText'] = self.NUMBERFORMAT % self.sliceSet.analyzer.zMin;
		p['zMaxText'] = self.NUMBERFORMAT % self.sliceSet.analyzer.zMax;
		
		resultDoc = top.substitute(p);
		
		for layer in self.layers:
			p['zLevel'] = layer.zLevel();
			p['layerNo'] = layer.slice.layerNo;
			p['xTransform'] = layer.xTransform();
			p['yTransform'] = layer.yTransform();
			p['path'] = layer.path();
			resultDoc += layer.subsitute(p);

		resultDoc += bottom.substitute(p);
		return resultDoc;
		
######
# Decorates a slice to provide extra computations for SVG Presentation.
# Needed only for SVG display
######	
class SVGLayer:
	def __init__(self,slice,unitScale,numformat):
		self.slice = slice;
		self.margin = 20;
		self.unitScale = unitScale;
		self.NUMBERFORMAT = numformat;
		self.paths = [];
		
	def zLevel(self):
		return self.NUMBERFORMAT % self.slice.zLevel;
		
	def xTransform(self):
		return self.margin;
		
	def yTransform(self):
		return (self.slice.layerNo + 1 ) * (self.margin + ( self.slice.sliceHeight * self.unitScale )) + ( self.slice.layerNo * 20 );
		
	def path(self):
		return self.paths.join("");
	
	