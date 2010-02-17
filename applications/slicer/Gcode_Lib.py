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
GCODE_NUMBER_FORMAT = "%0.2f";
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
	print "x=%0.4f,y=%0.4f,z=%0.4f" % (point.X(), point.Y(), point.Z());
	
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
		
	def getResults(self):
		return self.cmdBuffer;
	
	def addCommand(self,command):
		self.cmdBuffer.append(command);
	
	def comment(self,cmnt):
		self.cmdBuffer.append("(" + cmnt + ")");
		
	def reset(self):
		
		self.currentX=None;
		self.currentY=None;
		self.currentZ=None;
		self.currentFeed=None;
		self.cmdBuffer = [];
		self.lastMove = None;
		
	"""
		Follow an edge. A command will be issued ( if necessary ) to move
		to the beginning of the edge, then to the end along the commanded edge.
		
		Usually edges will be chained together so that the end of one is the
		same of the beginning of the next.
	"""
	def followEdge(self,edge,feed):
		range = Brep_Tool.Range(edge);
		logging.debug( "Edge Bounds:" + str(range) );
		
		curve = BRepAdaptor.BRepAdaptor_Curve(edge);		
		p1 = curve.FirstParameter();
		p2 = curve.LastParameter();
		curveType = curve.GetType();
		
		logging.debug(  "Edge is a curve of type:" + str(curveType));
		
		#get first and last points.
		firstPoint = gp.gp_Pnt();
		lastPoint = gp.gp_Pnt();
		BRepLProp_CurveTool.Value(curve,p1,firstPoint );
		BRepLProp_CurveTool.Value(curve,p2,lastPoint );	

		#first, rapid to the beginning, if we are not already there
		self.movePt(firstPoint,feed,"G00");
		
		if ( curveType == 0 ):

			self.movePt(lastPoint,feed);
		
		if ( curveType == 1 ):
			#a circle.
			#print "FirstPoint="
			#printPoint(firstPoint);
			
			##print "LastPoint="
			#printPoint(lastPoint);			
			circle = curve.Circle();
			center = circle.Location();
			
			axisDir = circle.Axis().Direction(); #need this to determine which way the arc goes			
			#assume we are essentially in 2d space, so we're looking only at wehter the
			#axis of the circle is +z or -z
			zDir = gp.gp().DZ();
			if zDir.IsEqual(axisDir,TOLERANCE):
				c = "G03"
				#print "detected ccw arc";
			else:
				#print "detected cw arc";
				c = "G02";			
			
			#printPoint(center);
			#TODO: handle incremental coordinates
			#how do we know whether we want to do a clockwise or counterclockwise arc?
			self.arc(center.X()-firstPoint.X(),center.Y()-firstPoint.Y(),lastPoint.X(),lastPoint.Y(),lastPoint.Z(),feed,c);

						
		if ( curveType == 2 ):
			#an ellipse
			self.addCommand("G02 Ellipse Not Implemented");	
	
	#follow the segments in a wire
	def followWire(self,wire, feed ):
		logging.debug( "Following Wire:" + str(wire));
		bwe = BRepTools.BRepTools_WireExplorer(wire);
		
		while bwe.More():
			edge = bwe.Current();
			self.followEdge(edge,feed);
			bwe.Next();
		bwe.Clear();

		
	#follow a compound
	def followCompound(self,compound,feed):
		texp = TopExp.TopExp_Explorer();
		texp.Init(compound,TopAbs.TopAbs_WIRE);

		while ( texp.More() ):
			wire = ts.Wire(texp.Current());
			self.followWire(wire,feed);
			texp.Next();	
		texp.ReInit();

	#follow a shape
	def followShape(self,shape,feed):
		if shape.ShapeType() == TopAbs.TopAbs_WIRE:
			self.followWire(ts.Wire(shape),feed);
		elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
			self.followCompound(ts.Compound(shape),feed);
		elif shape.ShapeType() == TopAbs.TopAbs_EDGE:
			self.followEdge(shape,feed);
		else:
			print "Cannot follow shape";
	
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
			command.append( ("I" + GCODE_NUMBER_FORMAT)  % (i ));
		
		if j != None and j != 0:
			command.append( ("J" + GCODE_NUMBER_FORMAT)  % (j ));

		if x != None and x != self.currentX:
			command.append( ("X" + GCODE_NUMBER_FORMAT)  % (x ));
			self.currentX = x;
		if y != None and y != self.currentY:
			command.append( ("Y" + GCODE_NUMBER_FORMAT)  % (y ));
			self.currentY = y;
			
		if z != None and z != self.currentZ:
			command.append( ("Z" + GCODE_NUMBER_FORMAT)  % (z ));
			self.currentZ = z;
			
		if feed != None and  feed != self.currentFeed:
			command.append( ("F" + GCODE_NUMBER_FORMAT)  % (feed ));
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
			cmds.append( ("X" + GCODE_NUMBER_FORMAT)  % (x) );
			self.currentX = x
		if y != self.currentY:
			cmds.append( ("Y" + GCODE_NUMBER_FORMAT)  % (y) );
			self.currentY = y
		if z != self.currentZ:
			cmds.append( ("Z" + GCODE_NUMBER_FORMAT)  % (z) );
			self.currentZ = z
		if feed != self.currentFeed:
			cmds.append(("F" + GCODE_NUMBER_FORMAT) % (feed) );
			self.currentFeed = feed;	
			
		self.addCommand( " ".join(cmds));

		
	def rapid(self,x = None, y = None, z = None,  feed=None ):
		self.move(x,y,z,feed,"G00");
		
	def start(self):
		self.addCommand(self.startUp);
		
	def end(self):
		self.addCommand( "M2");
		

		
	