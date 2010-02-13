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

	
"""
	State machine for creating gcode.
	This class will track the current position, which
	makes it easier to move along a particular path.
	
	Each command returns a list of commands that correspond to
	the desired action, or an empty list if no action is needed

"""
class GCode_Generator:
	def __init__(self):
	
		self.reset();
		self.startUp = "G90 ";
	
	
	def reset(self):
		self.currentX=None;
		self.currentY=None;
		self.currentZ=None;
		self.currentFeed=0.0;
		
	#set the current position of the machine
	#without executing a move to get there
	def setPosition(x,y,z):
		self.currentX= x;
		self.currentY=y;
		self.currentZ=z;
		
	"""
		Follow an edge. A command will be issued ( if necessary ) to move
		to the beginning of the edge, then to the end along the commanded edge.
		
		Usually edges will be chained together so that the end of one is the
		same of the beginning of the next.
	"""
	def followEdge(self,edge,feed):
		cmds = [];
		range = Brep_Tool.Range(edge);
		logging.debug( "Edge Bounds:" + str(range) );
		
		curve = BRepAdaptor.BRepAdaptor_Curve(edge);		
		p1 = curve.FirstParameter();
		p2 = curve.LastParameter();
		curveType = curve.GetType();
		
		logging.debug(  "Edge is a curve of type:" + str(curveType));
		if ( curveType == 0 ):
			pt1 = gp.gp_Pnt();
			pt2 = gp.gp_Pnt();
			BRepLProp_CurveTool.Value(curve,p1,pt1 );
			BRepLProp_CurveTool.Value(curve,p2,pt2 );
			#a line. move to the beginning of the line, then
			#to the end.
			cmds.extend( self.movePt(pt1,feed));
			cmds.extend( self.movePt(pt2,feed));
			return cmds;
		
		if ( curveType == 1 ):
			return ["G02 Circle Not Implemented"];
						
		if ( curveType == 2 ):
			#an ellipse
			return ["G02 Ellipse Not Implemented"];	
	
	#follow the segments in a wire
	def followWire(self,wire, feed ):
		logging.debug( "Following Wire:" + str(wire));
		bwe = BRepTools.BRepTools_WireExplorer(wire);
		
		commands = [];
		while bwe.More():
			edge = bwe.Current();
			commands.extend ( self.followEdge(edge,feed) );
			bwe.Next();
		bwe.Clear();

		return commands;
		
	def movePt(self, gp_pt, feed ):
		return self.move(gp_pt.X(),gp_pt.Y(),gp_pt.Z(),feed);
	
	"""
		return the gcode required to move to the provided point.
		If no gcode is required, "" is returned instead
	
	"""
	def move(self, x = None, y = None, z = None,  feed=None, cmd="G01"):

		if x == None: x = self.lastx
		if y == None: y = self.lasty
		if z == None: z = self.lastz
		if feed == None: feed = self.currentFeed;
		
		#if there is no move at all, return immediately
		if ( x == self.currentX and y == self.currentY and z == self.currentZ ):
			logging.debug(  "No move required" );
			return "";
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
			
		return [" ".join(cmds)];

		
	def rapid(self,x = None, y = None, z = None,  feed=None ):
		return self.move(x,y,z,feed,"G00");
		
	def start(self):
		return [self.startUp];
		
	def end(self):
		return ["M2"];
		

		
	