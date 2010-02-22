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

###Logging Configuration
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s',
                    stream=sys.stdout)
					
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
TOLERANCE=0.0001;

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
class GCode_Generator:
	def __init__(self):
		
		self.reset();
		self.startUp = "G90 ";
		self.safeHeight = "3";
		self.numberFormat = DEFAULT_NUMBER_FORMAT;
		self.verbose = False;
	def getResults(self):
		return self.cmdBuffer;
	
	def addCommand(self,command):
		self.cmdBuffer.append(command);
	
	def debugCurrentPosition(self):
		if self.currentX and self.currentY and self.currentZ:
			print "Current Position: X=%0.4f,Y=%0.4f,Z=%0.4f" % (self.currentX, self.currentY, self.currentZ);
		else:
			print "Current Position is undefined."
		
	def comment(self,cmnt):
		self.addCommand("(" + cmnt + ")");
		
	def reset(self):
		
		self.currentX=None;
		self.currentY=None;
		self.currentZ=None;
		self.currentFeed=None;
		self.cmdBuffer = [];
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
		print str(edgeWrapper);
		if self.verbose:
			self.comment(str(edgeWrapper));
		
		#check to see if we should reverse this edge. Perhaps reversing it will work better.
		if not self.isPointClose(edgeWrapper.firstPoint,0.001):
			#print "moving to start."
			self.movePt(edgeWrapper.firstPoint,feed,"G00");		
		#else:
			#print "no move required, already at start."
		
		if edgeWrapper.isLine():
			self.movePt(edgeWrapper.lastPoint,feed);
		elif edgeWrapper.isCircle():
			circle = edgeWrapper.curve.Circle();
			center = circle.Location();
			
			axisDir = circle.Axis().Direction(); #need this to determine which way the arc goes			
			#assume we are essentially in 2d space, so we're looking only at wehter the
			#axis of the circle is +z or -z
			zDir = gp.gp().DZ();
			
			if edgeWrapper.reversed:
				zDir = zDir.Reversed();
				
			if zDir.IsEqual(axisDir,TOLERANCE):
				c = "G03"
				#print "detected ccw arc";
			else:
				#print "detected cw arc";
				c = "G02";			
			
			#TODO: handle incremental coordinates
			self.arc(center.X()-edgeWrapper.firstPoint.X(),center.Y()-edgeWrapper.firstPoint.Y(),edgeWrapper.lastPoint.X(),edgeWrapper.lastPoint.Y(),edgeWrapper.lastPoint.Z(),feed,c);
		else:
			self.addCommand("Curve Type %d Not implemented" % edgeWrapper.curveType );
		if self.verbose:
			self.comment("End follow Edge.");
	
		
	#follow the segments in a wire
	#wire is a WireWrapper object
	def followWire(self,wireWrapper, feed ):
		#logging.debug( "Following Wire:" + str(wireWrapper));
		#bwe = BRepTools.BRepTools_WireExplorer(wireWrapper.wire);
		
		for ew in wireWrapper.edgeList:
			self.followEdge(ew,feed );
	
	"""
		An arc. cmd is either G02 or G03
		i and j are the locations of the center of the arc
		x and y are the endpoints of the arc.
		
		Note that other form of g02/g03 arcs are not supported, since they are dangerous for
		very small arc lengths.
	"""
	def arc(self, i= None, j=None, x=None, y=None, z=None, feed=None, cmd="G02" ):
		
		#move
		command = [cmd];
		#todo: handle incremental coordinates. for now assume absolute coordinates
		#careful-- i and j are offsets, not coordinates.
		
		if i != None and i != 0:
			command.append( ("I" + self.numberFormat)  % (i ));
		
		if j != None and j != 0:
			command.append( ("J" + self.numberFormat)  % (j ));

		if x != None and x != self.currentX:
			command.append( ("X" + self.numberFormat)  % (x ));
			self.currentX = x;
		if y != None and y != self.currentY:
			command.append( ("Y" + self.numberFormat)  % (y ));
			self.currentY = y;
			
		if z != None and z != self.currentZ:
			command.append( ("Z" + self.numberFormat)  % (z ));
			self.currentZ = z;
			
		if feed != None and  feed != self.currentFeed:
			command.append( ("F" + self.numberFormat)  % (feed ));
			self.currentFeed = feed;
			

		self.addCommand( " ".join(command));
		#naive move tracking doesnt apply to arcs
		self.lastMove = None;
	
	def movePt(self, gp_pt, feed,cmd="G01" ):
		self.move(gp_pt.X(),gp_pt.Y(),gp_pt.Z(),feed,cmd);
	
	"""
		return the gcode required to move to the provided point.
		If no gcode is required, "" is returned instead
	
	"""
	def move(self, x = None, y = None, z = None,  feed=None, cmd="G01"):
		
		#self.debugCurrentPosition();
		
		if x == None: x = self.currentX
		if y == None: y = self.currentY
		if z == None: z = self.currentZ
		if feed == None: feed = self.currentFeed;
		
		#if there is no move at all, return immediately
		if ( x == self.currentX and y == self.currentY and z == self.currentZ ):
			logging.debug(  "No move required" );
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
					logging.debug("Caught a naive move. Adjusting to remove the extra");
					#TODO this approach only works with absolute coordinates
					#in incremental coordinates, we have to adjust the new vector to move the same
					#as all of the previous moves!
					self.cmdBuffer.pop();
					
			#store this one as the last move
			self.lastMove = proposedMove;
		
		#compare the direction of this move to the previous move.
		cmds=[]
		cmds.append(cmd);
		if x != self.currentX:
			cmds.append( ("X" + self.numberFormat)  % (x) );
			self.currentX = x
		if y != self.currentY:
			cmds.append( ("Y" + self.numberFormat)  % (y) );
			self.currentY = y
		if z != self.currentZ:
			cmds.append( ("Z" + self.numberFormat)  % (z) );
			self.currentZ = z
		if feed != self.currentFeed:
			cmds.append(("F" + self.numberFormat) % (feed) );
			self.currentFeed = feed;	
			
		self.addCommand( " ".join(cmds));
		#self.debugCurrentPosition();

		
	def rapid(self,x = None, y = None, z = None,  feed=None ):
		self.move(x,y,z,feed,"G00");
		
	def start(self):
		self.addCommand(self.startUp);
		
	def end(self):
		self.addCommand( "M2");
		

		
	