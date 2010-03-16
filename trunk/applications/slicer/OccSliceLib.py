"""
   Copyright [2009] [Dave Cowden ( dave.cowden@gmail.com)]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

	OccSliceLIb
	
	A library for using OCC to slice solids for 3d printing.
	Author: Dave Cowden, dave.cowden@gmail.com
	
	Features:
			Slice STEP and STL files by creating layers of faces
			Tolerate and fix bad STL files
			Provide viewer for slices and the target object
			Export an SVG slice file
	Installation:
	
			1)Install Python, version 2.5 or 2.6,			 
					GET:   	http://www.python.org/download/
					TEST:  	at a command prompt/console, type "python"-- you should get something like this:
										
								Python 2.5.2 (r252:60911, Feb 21 2008, 13:11:45) [MSC v.1310 32 bit (Intel)] on win32
								Type "help", "copyright", "credits" or "license" for more information
								>>>
			
			2)Install OpenCascade+pythonOCC. Follow instructions here:
			This script requires release wo-0.2 or more recent

					GET:		http://www.pythonocc.org/wiki/index.php/InstallWindows
					TEST:		at a python command prompt, type from OCC import *,
							   you should get no error, like this:
							   
								Python 2.5.2 (r252:60911, Feb 21 2008, 13:11:45) [MSC v.1310 32 bit (Intel)] o
								win32
								Type "help", "copyright", "credits" or "license" for more information.
								>>> from OCC import *
								>>>					
									
			3)Install the Cheetah Template library for python, version 2.2 from here: 
					GET:		http://sourceforge.net/project/showfiles.php?group_id=28961
					TEST:		at a python prompt try to import Cheetah, like this:
					
								Python 2.5.2 (r252:60911, Feb 21 2008, 13:11:45) [MSC v.1310 32 bit (Intel)] o
								win32
								Type "help", "copyright", "credits" or "license" for more information.
								>>> from Cheetah import *
								>>>		

			4)Copy OccSliceLib(this script) into a directory of your choice.
					TEST:		Run the script without any arguments to confirm installation is ok, and to print help:
			
								>python OccSliceLib.py
								>
								>OccSliceLib usage:
								   .....

"""


import os
import sys
import os.path
import wx
import logging
import time
import traceback

import math
import itertools

from OCC import STEPControl,TopoDS, TopExp, TopAbs,BRep,gp,GeomAbs,GeomAPI
from OCC import BRepBuilderAPI,BRepTools,BRepAlgo,BRepBndLib,Bnd,StlAPI,BRepAlgoAPI,BRepAdaptor
from OCC import BRepLProp,BRepGProp,BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepExtrema
from OCC import BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor,IntCurvesFace,Approx,BRepLib

from OCC.Display.wxDisplay import wxViewer3d
from OCC.Utils.Topology import Topo
from OCC.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.ShapeAnalysis import ShapeAnalysis_WireOrder
from OCC.ShapeFix import ShapeFix_Wire

from OCC import ShapeFix
from OCC import BRepBuilderAPI
from OCC import TopTools

from OCC import gce
#my libraries
#import Gcode_Lib
from Wrappers import *

import cProfile
import pstats

###Logging Configuration
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
                    stream=sys.stdout)

log = logging.getLogger('slicer');
log.setLevel(logging.INFO);

					
##
##  TODO:
##   X change pathmanager to allow other formats and easy structure access
##   X use builtin wxDisplay
##   X change naming of slice() to something else
##   X recognize .stp in addition to .step
##   X install guide
##   X remove unneed dumpTopology and shapeDescription
##     add 2d per-slice view on select of slice
##   X add ability to easily select slice thickness
##     add separate display for original object and slices
##   X sew faces from crappy stl files into single faces somehow
##   X remove reference to profile import

"""
	##
	##  Performance Improvement Ideas
	## 
    Compute slice boundaries once instead of per layer
	Generate gcode as you go instead of storing in memory
	 
 
"""
mainDisplay = None;
debugDisplay=None;

#####
#utility class instances
#available to all methods
#####
brt = BRepTools.BRepTools();
btool = BRep.BRep_Tool();
ts = TopoDS.TopoDS();
topexp = TopExp.TopExp()
texp = TopExp.TopExp_Explorer();
BRepLProp_CurveTool = BRepLProp.BRepLProp_CurveTool();


#### some constants
UNITS_MM = "mm";
UNITS_IN = "in";
UNITS_UNKNOWN = "units";

	
def debugShape(shape):
	global debugDisplay;
	if debugDisplay:
		debugDisplay.showShape(shape);

def hideDebugDisplay():
	global debugDisplay;
	if debugDisplay:
		debugDisplay.Hide();

def showDebugDisplay():
	global debugDisplay;
	if debugDisplay:
		debugDisplay.Show(True);


def sortPoints(pointList):
	"Sorts points by ascending X"
	pointList.sort(cmp= lambda x,y: cmp(x.X(),y.X()));

def pointsFromWire(wire,spacing):
	"Makes a single parameterized adapter curve from a wire"	

	points = [];
	#make a parameterized approximation of the wire
	adaptor = BRepAdaptor.BRepAdaptor_CompCurve (wire);
	adaptor.SetPeriodic(True);
	gc = GCPnts.GCPnts_UniformAbscissa(adaptor,spacing,0.0001);
	
	if gc.IsDone():
		log.info("Computed %d Points Successfully for the wire." % gc.NbPoints() );
		for i in range(1,gc.NbPoints()):
			points.append(adaptor.Value(gc.Parameter(i)));

	#workaround: for some odd reason periodic curves do not honor the start and the end
	#so work around by removing the last point if it is too close to the first one
	if points[0].Distance(points[len(points)-1]) < spacing:
		points.pop();
		log.info("Correcing for GCPnts periodic curve bug");
		
	return points;
	
		
def makeWiresFromOffsetShape(shape):
	"get all the wires from the offset shape"
	resultWires = TopTools.TopTools_HSequenceOfShape();
	if shape.ShapeType() == TopAbs.TopAbs_WIRE:
		log.info( "offset result is a wire" );
		wire = ts.Wire(shape);
		#resultWires.append(wire);
		resultWires.Append(wire);
	elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
		log.info( "offset result is a compound");

		bb = TopExp.TopExp_Explorer();
		bb.Init(shape,TopAbs.TopAbs_WIRE);
		while bb.More():
			w = ts.Wire(bb.Current());
						
			#resultWires.append(w);
			resultWires.Append(w);#
			#debugShape(w);
			bb.Next();
		
		bb.ReInit();	
	
	return resultWires;	


def checkMinimumDistanceForOffset(offset,resolution):
	"PERFORMANCE INTENSIVE!!!!"
	"check an offset shape to make sure that it does not overlap too closely"
	"this consists of making sure that none of the wires are too close to each other"
	"and that no individual wires have edges too close together"
	log.info ("Checking this offset for minimum distances");

	te = TopExp.TopExp_Explorer();
	resultWires = TopTools.TopTools_HSequenceOfShape();
	te.Init(offset,TopAbs.TopAbs_WIRE);
	
	allPoints = [];
	while te.More():
			w = ts.Wire(te.Current());
			wr = Wire(w);
			resultWires.Append(w);
			allPoints.extend(pointsFromWire(w,resolution*2));
			#for p in wr.discretePoints(resolution/2):
			#	debugShape(make_vertex(p));
			#	allPoints.append(p);
			te.Next();
	te.ReInit();
	
	#for p in allPoints:
	#	debugShape(make_vertex(p));
		
	log.info("There are %d wires, and %d points" % (resultWires.Length(), len(allPoints) ));
	
	#cool trick here: list all permutations of these points
	"this is where we could probably really improve this algorithm"
	for (p1,p2) in list(itertools.combinations(allPoints,2)):
		d = p1.Distance(p2);
		if d < resolution:
			log.warn("Distance %0.5f is less than expected value" % d );
			return False;
		#else:
			#log.info("Computed distance = %0.5f" % d );
			
	return True;


class AppFrame(wx.Frame):
  def __init__(self, parent,title,x,y):
      wx.Frame.__init__(self, parent=parent, id=-1, title=title, pos=(x,y),style=wx.DEFAULT_FRAME_STYLE,size = (400,300))
      self.canva = wxViewer3d(self);
  def showShape(self,shape):
		self.canva._display.DisplayShape(shape)
  def eraseAll(self):
  		self.canva._display.EraseAll();
	
		
class EdgeFollower:
	"returns edges in head-to-tail order, based on which edge is closest"
	"edges are returned so that their orientation is correct"
	"this is the core for the hatching algorithm"
	def __init__(self,infillEdges,boundaryEdges):
	
		self.infillEdges  = [];
		self.infillEdges.extend(infillEdges);
		
		self.boundaryEdges  = [];
		self.boundaryEdges.extend(boundaryEdges);
		
		self.lastEdgeIsInfill = False;
		self.lastEdge = None;
		self.tolerance = 0.0001;
		self.referencePoint = None;


	def _findEdgesSharingPoint(self,edgeList,point):
		"find all edges for which one end matches the specified point"
		"the returned tuples order the edge so its first point matches the supplied point"
		candidates  =[];
		for edge in edgeList:
			ew = Edge(edge);
			if point.Distance(ew.lastPoint) < self.tolerance:
				candidates.append([edge, ts.Edge(edge.Reversed())]);			
			if  point.Distance(ew.firstPoint ) < self.tolerance:
				candidates.append([edge, edge]);
		return candidates;
	
	def nextEdge(self):
		"get the next edge: returns Edge for the selected Edge"
		if not self.lastEdge:		
			e = self.infillEdges.pop();
			self.lastEdge = e;
			self.referencePoint = Edge(e).firstPoint;
			
			#sort infillEdges in ascending distance from the reference Point
			self.infillEdges.sort(cmp= lambda x,y: cmp(   
				Edge(x).firstPoint.Distance(self.referencePoint),
				Edge(y).firstPoint.Distance(self.referencePoint))		);			
			
			self.lastEdgeIsInfill=True;
			ew = Edge(e);
			log.debug( "First Edge",str(Point(ew.firstPoint)),str(Point(ew.lastPoint)));
			return e;
			
		sPoint = Edge(self.lastEdge).lastPoint;
		newEdge = None;
		
		if self.lastEdgeIsInfill:
			log.debug(  "we are looking for a boundary edge" );
			candidates = self._findEdgesSharingPoint(self.boundaryEdges,sPoint);
			if len(candidates) == 1:
				log.debug(  "only one match, so return him" );
				self.boundaryEdges.remove(candidates[0][0]);
				self.lastEdgeIsInfill = False;
				newEdge = candidates[0][1]
			elif len(candidates) == 0:
				log.debug(  "no boundary edges, so find a new starting infill edge" );
				newEdge = self.infillEdges.pop();
			else:
				log.debug(  "several candidates. pick the one whos endpoint is further away from the reference");
				candidates.sort(cmp = lambda x,y: cmp (
					Edge(x[1]).lastPoint.Distance(self.referencePoint),
					Edge(y[1]).lastPoint.Distance(self.referencePoint) )  );
				self.boundaryEdges.remove(candidates[0][0]);
				self.lastEdgeIsInfill = False;
				newEdge = candidates[0][1];
		else:
			log.debug( "we are looking for an infill edge" );
			candidates = self._findEdgesSharingPoint(self.infillEdges,sPoint);
			if len(candidates) == 0:
				log.debug( "no infill edges found matching this boundary. Select another Infill Edge" );
				self.lastEdgeIsInfill = True;
				newEdge = self.infillEdges.pop();
			elif len(candidates) == 1:
				log.debug( "only one choice" );
				self.lastEdgeIsInfill = True;
				self.infillEdges.remove(candidates[0][0]);
				newEdge = candidates[0][1];
			else:
				log.debug("several to choose from. return closest to reference point" );
				candidates.sort(cmp = lambda x,y: cmp (
					Edge(x[1]).lastPoint.Distance(self.referencePoint),
					Edge(y[1]).lastPoint.Distance(self.referencePoint) ) );
				candidates.reverse();
				self.infillEdges.remove(candidates[0][0]);
				self.lastEdgeIsInfill = True;
				newEdge = candidates[0][1];				
		
		self.lastEdge = newEdge;
		return newEdge;
		
	def hasMoreEdges(self):
		return len(self.infillEdges) > 0;


		
class BoundaryManager:
	"manages a set of boundaries and hatch intersections points"
	
	def __init__(self):
		self.intersectionPoints = {};
		self.wires = [];
		self.tolerance = 0.00001;
		self.approxCurves = {};
		
	def addIntersectionPoint(self, wire, point):
		"adds an intersection point for the specified wire"
		self.intersectionPoints[wire].append(point);
	
	def addWire(self, wire):
		self.wires.append(wire);
		self.intersectionPoints[wire] = [];
			
		#make a parameterized approximation of the wire
		adaptor = BRepAdaptor.BRepAdaptor_CompCurve (wire);
		curve = BRepAdaptor.BRepAdaptor_HCompCurve(adaptor);
		curveHandle = curve.GetHandle();

		#approximate the curve using a tolerance
		approx = Approx.Approx_Curve3d(curveHandle,self.tolerance,GeomAbs.GeomAbs_C2,2000,12);
		if approx.IsDone() and  approx.HasResult():
			# have the result
			anApproximatedCurve=approx.Curve();
			log.debug( "Curve is parameterized between %0.5f and %0.5f " % (  anApproximatedCurve.GetObject().FirstParameter(), anApproximatedCurve.GetObject().LastParameter() ));
			self.approxCurves[wire] = anApproximatedCurve;
			#builder =  BRepLib.BRepLib_MakeEdge(anApproximatedCurve);
			#self.showShape(builder.Edge());
		else:
			log.warn( "Failed to create approximation curve." );
			
	def buildEdges(self):
		"build all of the available edges on each boundary"
		edgeList = [];
		for [wire,curve] in self.approxCurves.iteritems():
			parameters = [];
			parameters.append(curve.GetObject().FirstParameter() );
			for point in  self.intersectionPoints[wire]:
				a = findParameterOnCurve(point,curve);
				#todo: check distance also
				parameters.append(a[1] );
			parameters.append(curve.GetObject().LastParameter() );
			parameters.sort();

			lastP = None;
			for p in parameters:
				if lastP:
					edgeList.append ( edgeFromTwoPointsOnCurve(curve,lastP,p ) );
				lastP = p;

		log.debug( "Made %d Boundary Edges from %d wires" % ( len(edgeList),len(self.approxCurves)));
		return edgeList;
		
	def makeEdge(self,p1, p2):
		"make an edge following a boundary as close as possible to the provided points"
		"the edge will always be along one of the nested curves. It is assumed that p1 and p2 lie on the same curve"		
		bestDistance = 999999;
		bestCurve = None;
		log.debug( "Searching %d wires for a fit for these points." % len(self.approxCurves));
		for [wire,curve] in self.approxCurves.iteritems():
			a = findParameterOnCurve(p1,curve);
			b = findParameterOnCurve(p2,curve);
			#print "Distance from curve is %05.f, %0.5f" % ( a[2], b[2] );
			return edgeFromTwoPointsOnCurve(curve,a[1],b[1]);
			
		print "Could not find a suitable curve. Are the points on any curve?"
		return None;
			
"""
	Class that provides easy access to commonly
	needed features of a solid shape
"""
class SolidAnalyzer:
	def __init__(self,shape):
		self.shape = shape;

		box = Bnd.Bnd_Box();
		b = BRepBndLib.BRepBndLib();
		b.Add(shape,box);
		
		self.bounds = box.Get();
		self.xMin = self.bounds[0];
		self.xMax = self.bounds[3];
		self.xDim = abs(self.xMax - self.xMin);
		self.yMin = self.bounds[1];
		self.yMax = self.bounds[4];
		self.yDim = abs(self.yMax - self.yMin);
		self.zMin = self.bounds[2];
		self.zMax = self.bounds[5];
		self.zDim = abs(self.zMax - self.zMin);
		
	"""
		Pretty print dimensions
	"""		
	def friendlyDimensions(self):		
		formatString = "x:%0.2f y:%0.2f z:%0.2f (" + self.guessUnitOfMeasure() + ")" 
		return formatString % ( self.xDim, self.yDim, self.zDim );
		
	"""
		Translate to positive space This returns another shape that
		has been translated into positive space the current shape and bounds are not modified:
		if you want to re-analyze the shape, create another analyzer from the new shape	
	"""	
	def translateToPositiveSpace(self):

		if self.xMin < 0 or self.yMin < 0 or self.zMin < 0:
			log.debug("Shape appears to be in negative space. Translating...");
			
			x = abs(self.xMin);
			y = abs(self.yMin);
			z = abs(self.zMin);
			p1 = gp.gp_Pnt(0,0,0);
			p2 = gp.gp_Pnt(x,y,z);
			xform = gp.gp_Trsf();
			xform.SetTranslation(p1,p2);
			log.info("Translating shape by x=%0.3f,y=%0.3f,z=%0.3f" % ( x, y, z ));
			bt = BRepBuilderAPI.BRepBuilderAPI_Transform(xform);
			bt.Perform(self.shape,False);
			return bt.Shape();
		else:
			log.debug("Translation is not required. Returning existing shape");
	
	"""
	  Given a list of dimenions, guess the unit of measure.
	  The idea is that usually, the dimenions are either in or mm, and its normally evident which one it is
	"""
	def guessUnitOfMeasure(self):
	
		dimList = [ abs(self.xMax - self.xMin ), abs(self.yMax - self.yMin ), abs(self.zMax - self.zMin) ];
		#no real part would likely be bigger than 10 inches on any side
		if max(dimList) > 10:
			return UNITS_MM;
	
		#no real part would likely be smaller than 0.1 mm on all dimensions
		if min(dimList) < 0.1:
			return UNITS_IN;
			
		#no real part would have the sum of its dimensions less than about 5mm
		if sum(dimList) < 10:
			return UNITS_IN;
		
		return UNITS_UNKNOWN;	

		
"""
	Slicing Parameters Object stores the various options
	that are selected during slicing.
	
	This object is separated out because as these routines
	become more complex, it is expected that this obejct will grow in size
"""
class SliceOptions:
	def __init__(self):
	
		#global options
		self.zMin = None;
		self.zMax = None;
		self.translateToPositiveSpace=False;
		self.numSlices = None;
		self.resolution = None;
		self.sliceHeight = None;
		self.uom = None;
		self.DEFAULT_RESOLUTION = { UNITS_MM : 0.3, UNITS_IN : 0.012 };
		self.FIRST_LAYER_OFFSET = 0.0001;
		self.DEFAULT_NUMSHELLS = 5;
		self.inFillAngle = 30;
		self.hatchPadding = 0.15; # a percentage
		self.inFillSpacing=1;
		#per-slice options
		self.numShells = self.DEFAULT_NUMSHELLS;
		
"""
	a set of slices that together make a part
"""
class Slicer:
	def __init__(self,shape,options):
		"initialize the object"
		self.slices=[]
		self.shape = shape;
		self.analyzer = SolidAnalyzer(shape);
		
		self.options = options;		

		self.saveSliceFaces = True;				
		self.FIRST_LAYER_OFFSET = 0.00001;
		
		#display property if avaialable
		self.display = None;
		
		#use options and actual object to determine unit of measure and other values
		#unit of measure
		self.uom = self.analyzer.guessUnitOfMeasure();
		if options.uom:
			self.uom = options.uom;
		
		#nozzle resolution
		self.resolution = options.DEFAULT_RESOLUTION.get(self.uom);
		if options.resolution:
			self.resolution = options.resolution;
			
		#slicing boundaries
		self.zMin = self.analyzer.zMin;
		self.zMax = self.analyzer.zMax;
		
		if options.zMin and options.zMin > self.zMin:
			self.zMin = options.zMin;
			
		if options.zMax and options.zMax < self.zMax:
			self.zMax = options.zMax;
			
		#slice thickness: default to same as resolution
		zRange = (self.zMax - self.zMin );
		self.sliceHeight = self.resolution;
		self.numSlices = math.floor(zRange/self.sliceHeight);
		
		#alter if number of slices or a particular thickness is provided
		if options.sliceHeight:
			self.sliceHeight = options.sliceHeight;
			self.numSlices = math.floor(zRange/self.sliceHeight);
		elif options.numSlices:
			self.numSlices = options.numSlices;
			self.sliceHeight = zRange / self.numSlices;
					
		log.info("Object Loaded. Dimensions are " + self.analyzer.friendlyDimensions());
		
	def showShape(self,shape):
		if self.display:
			self.display.showShape(shape);

	def execute(self):
		"slices the object with the settings supplied in the options property"
		t = Timer();		
		log.info("Slicing Started.");
		
		reportInterval = round(self.numSlices/10);
		log.info( "Slice Thickness is %0.3f %s, %0d slices. " % ( self.sliceHeight,self.uom,self.numSlices ));
		
		#make slices
		zLevel = self.zMin + self.FIRST_LAYER_OFFSET;
		sliceNumber = 1;
		t2 = Timer();
		while zLevel < self.zMax:
			log.warn( "Creating Slice %0d, z=%0.3f " % ( sliceNumber,zLevel));
			slice = self._makeSlice(self.shape,zLevel);

			if slice != None:
				#for f in slice.faces:
				#	self.showShape(f);			
				self.slices.append(slice);
				#todo: this should probably be componentize: filling is quite complex.
				self._fillSlice(slice);

			zLevel += self.sliceHeight
			sliceNumber += 1;
			
			#compute an estimate of time remaining every 10 or so slices
			if reportInterval > 0 and sliceNumber % reportInterval == 0:
				pc = ( zLevel - self.zMin )/   ( self.zMax - self.zMin) * 100;
				log.warn("%0.0f %% complete." % (pc) );			

		log.warn("Slicing Complete: " + t.finishedString() );
		log.warn("Throughput: %0.3f slices/sec" % (sliceNumber/t.elapsed() ) );

		
	def _makeHatchLines(self,shape ,zLevel):
		"Makes a set of hatch lines that cover the specified shape"
		hatchEdges = [];
		
		#debugShape(shape);
		
		box = Bnd.Bnd_Box();
		b = BRepBndLib.BRepBndLib();
		b.Add(shape,box);
		[xMin, yMin , zMin, xMax,yMax,zMax  ] = box.Get();	
		#print 'bounds are',xMin,yMin,xMax,yMax
		#add some space to make sure we are outside the boundaries

		xMin = xMin - ( self.options.resolution);
		yMin = yMin - ( self.options.resolution);		 
		xMax = xMax + (self.options.resolution );
		yMax = yMax + (self.options.resolution) ;

		global mainDisplay;

		wires = [];
		#compute direction of line
		lineDir = gp.gp_Dir( 1,math.cos(math.radians(self.options.inFillAngle)),zLevel );
		angleRads = math.radians(self.options.inFillAngle);
		xSpacing = self.options.inFillSpacing / math.sin(angleRads);		

		#tan theta = op/adj , adj =op / tan 
		xStart = ( xMin - (yMax - yMin)/math.tan(angleRads));
		xStop = xMax;	

		for xN in frange6(xStart,xStop,xSpacing):
			
			#make an edge to intersect
			p1 = gp.gp_Pnt(xN,yMin,zLevel);
			p2 = gp.gp_Pnt(xMax, (xMax - xN)* math.tan(angleRads) + yMin, zLevel);
		
			edge = edgeFromTwoPoints(p1,p2);
			#debugShape(edge);
			hatchEdges.append(edge);
		
		return hatchEdges;
		
	def _hatchSlice(self,slice,lastOffset):
		"take the a slice and compute a list of infillWires"
		log.debug( "Hatching Face...." );
		#debugShape(lastOffset);
		#a set of hatch lines that will conver the entire part
		#self.showShape(lastOffset);
		hatchEdges = self._makeHatchLines(lastOffset,slice.zLevel);
		log.debug( "I have %d Hatch Edges" % len(hatchEdges));
				
		#approximate each boundary by a parameterized bspine curve
		boundaryWires = makeWiresFromOffsetShape(lastOffset);
		
		#self.showShape(lastOffset);
		bm = BoundaryManager();
		#for wire in boundaryWires:
		for i in range(1,boundaryWires.Length()+1):
			wire = ts.Wire(boundaryWires.Value(i));
			#self.showShape(wire);
			bm.addWire(wire);

		infillEdges = [];
		boundaryEdges = [];
		
		boundariesFound = {};
		continueHatching = True;
		
		#for each generated hatch, intersect with each boundary
		for hatchLine in hatchEdges:
			if not continueHatching:
				break;
			else:
				log.debug( "Moving to next Hatch Line");
			hatchLineItersectionPoints = []
			log.debug( "There are %d boundary wires" % boundaryWires.Length());
			#for boundary in boundaryWires:
			for i in range(1,boundaryWires.Length()+1):
				boundary = ts.Wire(boundaryWires.Value(i));
				log.debug("Wire start");
				#debugShape(boundary);
				#time.sleep(.5);
				brp = BRepExtrema.BRepExtrema_DistShapeShape();
				#debugShape(hatchLine);
				#time.sleep(.2);
				brp.LoadS1(boundary);
				brp.LoadS2(hatchLine );

				if brp.Perform() and brp.Value() < 0.001:
					boundariesFound[boundary] = True;
					
					#number of intersections must be even for closed shapes
					#note that we want to avoid lines that are inside of islands.
					pointList = [];

					for k in range(1,brp.NbSolution()+1):
						hatchLineItersectionPoints.append(brp.PointOnShape1(k));
						bm.addIntersectionPoint(boundary,brp.PointOnShape1(k));

					for x in hatchLineItersectionPoints:
						log.debug( str(Point(x)));
				else:
					log.debug( "No Intersections Found");
			
			if len(hatchLineItersectionPoints) == 0 and (len(boundariesFound) ==  boundaryWires.Length()):
				continueHatching = False;
			
			#sort the points by ascending X, add to list of hashEdges
			hatchLineItersectionPoints.sort(cmp= lambda x,y: cmp(x.X(),y.X()));

			#HACK: dont know how to handle intersections at vertices or
			#tangent lines. for now, just ignore completely these.
			if len(hatchLineItersectionPoints) % 2 == 1:
				print "Detected Odd Number of intersection points. This is ignored for now."
				continue;
			#build infillEdges. We do this here so that we know which order to 
			#connect them. We are using to our advantage that the hatch line createse
			#a list of intersection points with increasing x values.
			i = 0;
			while i< len(hatchLineItersectionPoints):
				p1 = hatchLineItersectionPoints[i];
				p2 = hatchLineItersectionPoints[i+1];
				i += 2;
				
				e = edgeFromTwoPoints(p1,p2);
				if e:
					infillEdges.append(e);			

			
		#basically, alternate between an internal and and external edges
		if len(infillEdges) == 0:
			log.debug( "No Infill Edges Were Created. Returning" );
			return None;
		log.debug( "There are %d infill edges" % ( len(infillEdges)));
		
		boundaryEdges = bm.buildEdges();		

		follower = EdgeFollower(infillEdges,boundaryEdges);
		
		#build wires from the edges
		edgeList  = TopTools.TopTools_HSequenceOfShape();
		edgeList = [];
		
		while follower.hasMoreEdges():
			e = follower.nextEdge();
			#debugShape(e);
			edgeList.append(e );

		#TODO: because of some tolerance issue, i cannot seem to build a wire out
		#of these edges. but that's ok, there's really no reason why we cannot just use a 
		#sequence of edges instead
		#log.info("Building Wires from % Edges..." %  edges.Length() );
		#wireBuilder.ConnectEdgesToWires(edges.GetHandle(),0.01,True,resultWires.GetHandle() );
		#log.info ("Finished: Created %d wires" % resultWires.Length() );
		
		log.info("Finished Following hatching.");
		return edgeList;

			
	#fills a slice with the appropriate toolpaths.
	#returns: void
	#a number of wires are added to the slice as paths.
	def _fillSlice(self,slice):
	
		log.info("Filling Slice at zLevel %0.3f" % slice.zLevel);
		for f in slice.faces:
			numShells = 0;
			t3=Timer();
			shells = [];
			
			#attempt to create the number of shells requested by the user, plus one additional for infill
			#regardless, the last successfully created offset is used for hatch infill
			lastOffset = None;
			while numShells < self.options.numShells+1 :
				offset = (-1)*self.resolution*numShells;	
				try:
					#print "Offsetting at zlevel %0.3f" % slice.zLevel
					newOffset = self._offsetFace(f,offset);
					if lastOffset:
						t = Timer();
						r = checkMinimumDistanceForOffset(newOffset,self.options.resolution);						
						log.info("Finished checking offset dims. %0.2f elapsed." % t.elapsed() );
						if not r:
							log.warn("Shell is too close to other shells.");
							break;
							
					#sa = SolidAnalyzer(newOffset);
					#if min([sa.xDim,sa.yDim]) < ( self.resolution ):
					#	log.warn("Resulting Shape is too small to produce. Skipping this shell.");
					#	#raise Exception,"Resulting Shape is too small to produce."
					#	break;
					lastOffset = newOffset
					#debugShape(newOffset);
					shells.append(newOffset);
					log.info('Created Shell Successfully.');
				except Exception as e:
					traceback.print_exc(file=sys.stdout);
					log.warn(e.args);
					#raise e;
				numShells+=1;
			
			#completed
			if numShells < self.options.numShells:
				log.warn(("Could only create %d of the requested %d shells:") % (numShells, self.options.numShells ));
			
			#the last shell is for hatch rather than infill.
			#if it was possible to create the last shell, that means it is large enough to fill
			lastShell = shells.pop();
			
			#add shells to the slice
			for s in shells:
				slice.fillWires.extend(
					listFromHSequenceOfShape(
						makeWiresFromOffsetShape(s),TopAbs.TopAbs_WIRE));
			
			#use last shell for hatching
			#debugShape(lastShell);
			infillEdges = self._hatchSlice(slice,lastShell);
			slice.fillEdges.extend(infillEdges);
			
			for w in slice.fillWires:
				debugShape(w);
			
			for e in slice.fillEdges:
				debugShape(e);
				
		
		log.info("Filling Complete, Created %d paths." % len(slice.fillWires)  );


		
	#offset a face, returning the offset shape
	def _offsetFace(self,face,offset ):
		resultWires = [];
		ow = brt.OuterWire(face);
		bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
		bo.AddWire(ow);

		#now get the other wires
		te = TopExp.TopExp_Explorer();
		
		te.Init(face,TopAbs.TopAbs_WIRE);
		while te.More():
				w = ts.Wire(te.Current());
				if not w.IsSame(ow):
						bo.AddWire(w);
				te.Next();
		te.ReInit();
			
		bo.Perform(offset,0.00001);
		if  not bo.IsDone():
			raise Exception, "Offset Was Not Successful.";
		else:
			return  bo.Shape();
			
	def _makeSlice(self,shapeToSlice,zLevel):
		s = Slice();
		
		#change if layers are variable thickness
		s.sliceHeight = self.sliceHeight;		
		s.zLevel = zLevel;

		#make a cutting plane
		p = gp.gp_Pnt ( 0,0,zLevel );
			
		origin = gp.gp_Pnt(0,0,zLevel-1);
		csys = gp.gp_Ax3(p,gp.gp().DZ())
		cuttingPlane = gp.gp_Pln(csys);	
		bff = BRepBuilderAPI.BRepBuilderAPI_MakeFace(cuttingPlane);
		face = bff.Face();
		
		#odd, a halfspace is faster than a box?
		hs = BRepPrimAPI.BRepPrimAPI_MakeHalfSpace(face,origin);
		hs.Build();	
		halfspace = hs.Solid();
				
		#make the cut
		bc = BRepAlgoAPI.BRepAlgoAPI_Cut(shapeToSlice,halfspace);
		cutShape = bc.Shape();
		
		#search the shape for faces at the specified zlevel
		texp = TopExp.TopExp_Explorer();
		texp.Init(cutShape,TopAbs.TopAbs_FACE);
		foundFace = False;
		while ( texp.More() ):
			face = ts.Face(texp.Current());
			if self._isAtZLevel(zLevel,face):
				foundFace = True;
				log.debug( "Face is at zlevel" + str(zLevel) );
				s.addFace(face);
			texp.Next();
		
		#free memory
		face.Nullify();
		bc.Destroy();
		texp.Clear();
		texp.Destroy();
			
		if not foundFace:
			log.warn("No faces found after slicing at zLevel " + str(zLevel) + " !. Skipping This layer completely");
			return None;
		else:				
			return s;		
		
	def _isAtZLevel(self,zLevel,face):
		bf = BRepGProp.BRepGProp_Face(face);
		bounds = bf.Bounds();
		vec = gp.gp_Vec();
		zDir = gp.gp().DZ();
		pt = gp.gp_Pnt();
		#get a normal vector to the face
		bf.Normal(bounds[0],bounds[1],pt,vec);
		z=pt.Z();	
		sDir = gp.gp_Dir(vec);
		return (abs(z - zLevel) < 0.0001 ) and ( zDir.IsParallel(sDir,0.0001))


"""
	One slice in a set of slices that make up a part
	A slice typically has one or more faces, one or more
	outer boundaries, and other infill paths

"""
class Slice:
	def __init__(self):
		log.debug("Creating New Slice...");
		self.path = "";
		
		#actually these are Wires
		self.fillWires = [];
		self.fillEdges = [];
		self.boundaryWires = [];
		self.zLevel=0;
		self.zHeight = 0;
		self.layerNo = 0;
		self.sliceHeight=0;
		self.faces = [];
		
	"TODO: Do we really need this?"
	def addFace(self, face ):
		copier = BRepBuilderAPI.BRepBuilderAPI_Copy(face);
		self.faces.append(ts.Face(copier.Shape()));
		copier.Delete();
	
	def getBounds(self):
		"Get the bounds of all the faces"
		t = Timer();
		box = Bnd.Bnd_Box();
		b = BRepBndLib.BRepBndLib();	
		for face in self.faces:
			b.Add(face,box);
			
		bounds = box.Get();
		xMin = bounds[0];
		xMax = bounds[3];
		yMin = bounds[1];
		yMax = bounds[4];
		zMin = bounds[2];
		zMax = bounds[5];
		return [xMin,xMax,yMin,yMax,zMin,zMax];

	
		

		
	
"""
	Writes a sliceset to specified file in Gcode format.	
	Accepts a slicer, and exports gcode for the slices that were produced by the slicer.
	The slicer has already sliced and filled each layer. This class simply converts
	these toolpaths to Gcode
	
"""		
class GcodeExporter ():
	def __init__(self):

		self.description="Description";
		self.feedRate = 100.0;
		self.numberFormat = "%0.3f";
		self.verbose =False;
		self.display = None;
	
	def export(self, sliceSet, fileName):
		
		slices = sliceSet.slices;
		msg = "Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...";
		log.info("Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...");
		print msg;
		
		gcodewriter = Gcode_Lib.GCode_Generator();
		gcodewriter.verbose = self.verbose;
		gcodewriter.numberFormat = self.numberFormat;
		gcodewriter.start();
		gcodewriter.comment(self.description);
		exportedPaths = 0;

		for slice in slices:
			print "zLevel %0.5f ...." % slice.zLevel,
			gcodewriter.comment('zLevel' + str(slice.zLevel) );
			
			log.info( "zLevel %d, %d offset paths,%d fill edges " % ( slice.zLevel, len(slice.fillWires), len(slice.fillEdges) ));
			gcodewriter.comment("offsets");
			for fw in slice.fillWires:
				if self.verbose:
					gcodewriter.comment("begin wire");
				log.info(">>begin wire");
				#print "base wire:",str(fw);				
				#sw =fw.getSortedWire();
				#print "sorted wire:",str(sw);				
				gcodewriter.followWire(Wire(fw),self.feedRate);
				log.info(">>end wire");
				
				if self.verbose:
					gcodewriter.comment("end wire");				

				#if self.display:
				#	self.display.showShape(fw);
					
				exportedPaths += 1;


			gcodewriter.comment("infill");				
			for e in slice.fillEdges:
				if self.verbose:
					gcodewriter.comment("begin edge");
				gcodewriter.followEdge(Edge(e),self.feedRate);				
				if self.verbose:
					gcodewriter.comment("end edge");
				exportedPaths += 1;
			print "[Done]"
		commands = gcodewriter.getResults();
		f = open(fileName,'w');
		f.write("\n".join(commands));
		f.close()
		print "Gcode Export Complete."

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
	def zLevel(self):
		return self.NUMBERFORMAT % self.slice.zLevel;
		
	def xTransform(self):
		return self.margin;
		
	def yTransform(self):
		return (self.slice.layerNo + 1 ) * (self.margin + ( self.slice.sliceHeight * self.unitScale )) + ( self.slice.layerNo * 20 );
		
	def svgPathString(self):
		"return svg path string for a slice"

		
"""
	Read a shape from Step file
"""
def readStepShape(fileName):
	log.info("Reading STEP file:'" + fileName + "'...");
	stepReader = STEPControl.STEPControl_Reader();
	stepReader.ReadFile(fileName);
	
	numItems = stepReader.NbRootsForTransfer();
	numTranslated = stepReader.TransferRoots();
	log.info("Read " + str(numTranslated) + " from File.");
	shape = stepReader.OneShape();
	log.info("Done.");
	return shape;

def readSTLShape(fileName):
	ts = TopoDS.TopoDS();

	log.info("Reading STL:'" + fileName + "'...");
	#read stl file
	shape = TopoDS.TopoDS_Shape()
	stl_reader = StlAPI.StlAPI_Reader()
	stl_reader.Read(shape,fileName)
	log.info("Fixing holes and degenerated Meshes...");
	sf = ShapeFix.ShapeFix_Shape(shape);
	sf.Perform();
	fixedShape = sf.Shape();
	log.info("Making Solid from the Shell...");
	bb = ShapeFix.ShapeFix_Solid();
	return bb.SolidFromShell(ts.Shell(fixedShape));
	log.info("Done.");
	return bb.Solid();



def printUsage():
	print """
		Usage: OccSliceLib <inputfile: STL or STEP> 
			- inputfile [required] is an STL or STEP file, ending in .stl, .stp, or .step.
		
		Creates an SVG output file compatible with skeinforge	, in same directory as inputfile
	"""
def main(filename):

	global mainDisplay;
	global debugDisplay;

	app = wx.PySimpleApp()
	wx.InitAllImageHandlers()
	
	ok = False;
	if filename.lower().endswith('stl'):
		theSolid = readSTLShape(filename);
		ok = True;
	
	if filename.lower().endswith('step') or filename.lower().endswith('stp'):
		theSolid = readStepShape(filename);
		ok = True;
	
	if not ok:
		printUsage();
		return;
	
	#compute output filename
	#outFileName = filename[ : filename.rfind( '.' ) ] + '_sliced.svg'
	outFileName = filename[ : filename.rfind( '.' ) ] + '_sliced.nc'
	
	frame = AppFrame(None,filename,20,20)
	frame.canva.InitDriver()

	sliceFrame = AppFrame(None,"Slices of " + filename,420,20)
	sliceFrame.canva.InitDriver()
	sliceFrame.canva._display.SetModeWireFrame()
	sliceFrame.Show(True)
	mainDisplay = sliceFrame;
	
	gcodeFrame = AppFrame(None,"Generated Toolpaths" + filename,820,20)
	gcodeFrame.canva.InitDriver()
	gcodeFrame.canva._display.SetModeWireFrame()
	gcodeFrame.Show(True)	


	debugDisplay = AppFrame(None,"Debug Display",1220,20)
	debugDisplay.canva.InitDriver()
	debugDisplay.canva._display.SetModeWireFrame()
	debugDisplay.Show(True)	

	
	analyzer = SolidAnalyzer(theSolid);	
	shape = analyzer.translateToPositiveSpace();

	frame.showShape(shape);	
	frame.Show(True)			

	#slicing options: use defaults
	options = SliceOptions();
	options.numSlices=1;
	options.numShells=6;
	options.resolution=0.3;
	options.inFillAngle=45;
	options.inFillSpacing=.3;
	options.zMin=3
	options.zMax=7
	
	#slice it
	try:
		sliceSet = Slicer(shape,options);
		sliceSet.display = sliceFrame;	
		sliceSet.execute()
		
		#export to gcode
		#gexp = GcodeExporter();
		#gexp.description="Reprap Test";
		#gexp.display = gcodeFrame;
		#gexp.export(sliceSet, outFileName);
		#gexp.verbose = True;
		#cProfile.runctx('sliceSet.execute()', globals(), locals(), filename="slicer.prof")	;			
		
		#p = pstats.Stats('slicer.prof')
		#p.sort_stats('time')
		#p.print_stats(0.1);
	except:
		traceback.print_exc(file=sys.stdout);
		log.critical( 'Slicing Terminated.');	
		
	app.SetTopWindow(frame)
	app.MainLoop() 

	
if __name__=='__main__':
	nargs = len(sys.argv);

	if nargs > 1:
		filename = sys.argv[1];
		
		main(filename);
	else:
		printUsage();
		