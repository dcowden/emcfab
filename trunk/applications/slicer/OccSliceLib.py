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

from OCC import STEPControl,TopoDS, TopExp, TopAbs,BRep,gp
from OCC import BRepBuilderAPI,BRepTools,BRepAlgo,BRepBndLib,Bnd,StlAPI,BRepAlgoAPI
from OCC import BRepLProp,BRepGProp,BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepExtrema
from OCC import BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor,IntCurvesFace

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
import fillLib
import Gcode_Lib


###Logging Configuration
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(message)s',
                    stream=sys.stdout)

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

 
mainDisplay = None;

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

"""
	Utility class to provide timing information
"""		
class Timer:
	def __init__(self):
		self.startTime = time.time();
		self.startAscTime = time.asctime();
		
	def start(self):
		return self.startTime;
			
	def elapsed(self):
		end = time.time();
		return end - self.startTime;
		
	def finishedString(self):
		return "%0.3f sec" % ( self.elapsed() );
	

def printPoint(point):
	return "x=%0.4f,y=%0.4f,z=%0.4f" % (point.X(), point.Y(), point.Z());

def sortPoints(pointList):
	"Sorts points by ascending X"
	pointList.sort(cmp= lambda x,y: cmp(x.X(),y.X()));

def edgeFromTwoPoints(p1,p2):
	builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2);
	builder.Build();			
	edge = builder.Edge();
	return edge;
	
def make_edge(shape):
    spline = BRepBuilderAPI_MakeEdge(shape)
    spline.Build()
    return spline.Shape()	
	
#to override sometingselected,
# overrid Viewer3d.    
# def Select(self,X,Y):
#
#
class AppFrame(wx.Frame):
  def __init__(self, parent,title,x,y):
      wx.Frame.__init__(self, parent=parent, id=-1, title=title, pos=(x,y),style=wx.DEFAULT_FRAME_STYLE,size = (400,300))
      self.canva = wxViewer3d(self);
  def showShape(self,shape):
		self.canva._display.DisplayShape(shape)
  def eraseAll(self):
  		self.canva._display.EraseAll();

def frange6(*args):
    """A float range generator."""
    start = 0.0
    step = 1.0

    l = len(args)
    if l == 1:
        end = args[0]
    elif l == 2:
        start, end = args
    elif l == 3:
        start, end, step = args
        if step == 0.0:
            raise ValueError, "step must not be zero"
    else:
        raise TypeError, "frange expects 1-3 arguments, got %d" % l

    v = start
    while True:
        if (step > 0 and v >= end) or (step < 0 and v <= end):
            raise StopIteration
        yield v
        v += step		
		
"""
	EdgeWrapper provides additional functions and features
	for an OCC TopoDS_Edge object
"""
class EdgeWrapper:
	def __init__(self,edge):
		self.edge = edge;
		self.reversed = False;
		
		#get the first and last points for the underlying curve
		curve = BRepAdaptor.BRepAdaptor_Curve(edge);		
		p1 = curve.FirstParameter();
		p2 = curve.LastParameter();
		self.curve = curve;
		self.curveType = curve.GetType();
		#get first and last points.
		fP1 = gp.gp_Pnt();
		fP2 = gp.gp_Pnt();
		BRepLProp_CurveTool.Value(curve,p1,fP1 );
		BRepLProp_CurveTool.Value(curve,p2,fP2 );
	
		#set up the endpoints for easy access?
		if edge.Orientation() == TopAbs.TopAbs_REVERSED:
			self.reversed=True;
			self.firstPoint = fP2;
			self.lastPoint = fP1
		else:
			self.firstPoint = fP1;
			self.lastPoint = fP2;
			
	def isLine(self):
		return self.curveType == 0;
		
	def isCircle(self):
		return self.curveType == 1;
		
	def __str__(self):
		if self.isLine():
			s = "Line:\t";
		elif self.isCircle():
			s  = "Circle:\t";
		else:
			s = "Curve:\t";
		return s + printPoint(self.firstPoint) + "-->" + printPoint(self.lastPoint);

"""
    WireWrapper provides additional functions and features
	for an OCC TopoDS_Wire object
"""
class WireWrapper:
	def __init__(self,wire):
		self.wire = wire;
		
		#build a list of the edges
		self.edgeList = [];
		bwe = BRepTools.BRepTools_WireExplorer(wire);
		while bwe.More():
			edge = bwe.Current();
			self.edgeList.append(EdgeWrapper(edge));
			bwe.Next();
		bwe.Clear();		
	
	#sort the wire using ShapeFix to 
	#connect and re-order them head-to-tail
	#returns a wire for the constructed wire
	#or a reference to this object if there was a problem constructing it
	def getSortedWire(self,precision=0.001):
	
		sw = ShapeFix_Wire();
		sw.Load(self.wire);
		sw.ClosedWireMode = True;
		sw.FixReorderMode = 1;
		sw.FixConnectedMode = 1;
		sw.FixLackingMode = 1;
		sw.FixGaps3dMode = 1;
		sw.FixConnectedMode = 1;
		sw.FixDegeneratedMode = 1;

		
		if sw.Perform():
			#logging.info("WireSorting Was Successful!");
			return WireWrapper(sw.Wire());	
		else:
			#logging.info("WireSorting unsuccessful. returning self.");
			return self;

	def __str__(self):
		p = [];
		p.append("Wire (" + str(len(self.edgeList)) + "  edges ):");
		for e in self.edgeList:
			p.append("\t" + str(e) );
		
		return "\n".join(p);



	
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
			logging.debug("Shape appears to be in negative space. Translating...");
			
			x = abs(self.xMin);
			y = abs(self.yMin);
			z = abs(self.zMin);
			p1 = gp.gp_Pnt(0,0,0);
			p2 = gp.gp_Pnt(x,y,z);
			xform = gp.gp_Trsf();
			xform.SetTranslation(p1,p2);
			logging.info("Translating shape by x=%0.3f,y=%0.3f,z=%0.3f" % ( x, y, z ));
			bt = BRepBuilderAPI.BRepBuilderAPI_Transform(xform);
			bt.Perform(self.shape,False);
			return bt.Shape();
		else:
			logging.debug("Translation is not required. Returning existing shape");
	
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
		self.inFillSpacing=1;
		#per-slice options
		self.numShells = self.DEFAULT_NUMSHELLS;
		
"""
	a set of slices that together make a part
"""
class Slicer:
	def __init__(self,shape,options):
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
					
		logging.info("Object Loaded. Dimensions are " + self.analyzer.friendlyDimensions());
		
	def showShape(self,shape):
		if self.display:
			self.display.showShape(shape);

	def execute(self):

		t = Timer();		
		logging.info("Slicing Started.");
		
		reportInterval = round(self.numSlices/10);
		logging.info( "Slice Thickness is %0.3f %s, %0d slices. " % ( self.sliceHeight,self.uom,self.numSlices ));
		
		#make slices
		zLevel = self.zMin + self.FIRST_LAYER_OFFSET;
		sliceNumber = 1;
		t2 = Timer();
		while zLevel < self.zMax:
			logging.info( "Creating Slice %0d, z=%0.3f " % ( sliceNumber,zLevel));
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
				logging.info("%0.0f %% complete." % (pc) );			

		logging.info("Slicing Complete: " + t.finishedString() );
		logging.info("Throughput: %0.3f slices/sec" % (sliceNumber/t.elapsed() ) );

	#take the lastOffsetShape, and compute infill.
	def _hatchSlice(self,slice,lastOffset):
		[xMin,xMax,yMin,yMax,zMin,zMax] = slice.getBounds();
		global mainDisplay;

		wires = [];
		#compute direction of line
		lineDir = gp.gp_Dir( 1,math.cos(math.radians(self.options.inFillAngle)),slice.zLevel );
		angleRads = math.radians(self.options.inFillAngle);
		xSpacing = self.options.inFillSpacing / math.cos(angleRads);		

		#tan theta = op/adj , adj =op / tan 
		xStart = ( xMin - (yMax - yMin)/math.tan(angleRads));
		xStop = xMax;
		foundPart = False;
		
		wireBuilder = BRepBuilderAPI.BRepBuilderAPI_MakeWire ();

		for xN in frange6(xStart,xStop,xSpacing):
			
			#make an edge to intersect
			p1 = gp.gp_Pnt(xN,yMin,slice.zLevel);
			p2 = gp.gp_Pnt(xMax, (xMax - xN)* math.tan(angleRads), slice.zLevel);
		
			edge = edgeFromTwoPoints(p1,p2);
			
			#intersect with offset
			brp = BRepExtrema.BRepExtrema_DistShapeShape();
			brp.LoadS1(lastOffset);
			brp.LoadS2(edge );
			edges = [];
			if brp.Perform() and brp.Value() < 0.0001:
				foundPart = True;
				logging.debug ("Found %d Intersections" % brp.NbSolution());
				#number of intersections must be even for closed shapes
				#note that we want to avoid lines that are inside of islands.
				pointList = [];				
				i = 1;
				while i < brp.NbSolution()+1:
					pointList.append(brp.PointOnShape1(i));
					i+= 1;
				
				#sort the points by ascending X
				pointList.sort(cmp= lambda x,y: cmp(x.X(),y.X()));

				i = 0;
				while i< brp.NbSolution():
					p1 = pointList[i];
					p2 = pointList[i+1];
					i += 2;
					e = edgeFromTwoPoints(p1,p2);
					edges.append(e);
					self.showShape(e);
			else:
				##print "Could not find intersections.";
				if foundPart:
					#print "This edge did not intersect anything, Filling finished since i had already found it before"
					break;


			
		return wires;
		
	#fills a slice with the appropriate toolpaths.
	#returns: void
	#a number of wires are added to the slice as paths.
	def _fillSlice(self,slice):
	
		logging.info("Filling Slice at zLevel %0.3f" % slice.zLevel);
		for f in slice.faces:
			currentOffset = 1;
			#self.display.showShape(f);
			t3=Timer();
			lastOffset = None;
			while currentOffset < self.options.numShells:
				offset = (-1)*self.resolution*currentOffset;	
				#print "Computing Offset.."
				s = self._offsetFace(f,offset);
				self.showShape(s);
				if s:
					lastOffset = s;					
					slice.fillWires.extend(self._makeWiresFromOffsetShape(s));
				currentOffset+=1;
			#ok now the last shell is available to serve as the boundary for
			#our hatching also
			#take the last wire(s) created 
			print "Hatching Sliced Items.."
			infillWires = self._hatchSlice(slice,lastOffset);

			slice.fillWires.extend(infillWires );
			
		logging.info("Filling Complete, Created %d paths." % len(slice.fillWires)  );
		


		
	def _makeWiresFromOffsetShape(self,shape):
		resultWires = [];
		if shape.ShapeType() == TopAbs.TopAbs_WIRE:
			#print "offset result is a wire"
			wire = ts.Wire(shape);
			resultWires.append(WireWrapper(wire));
		elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
			#print "offset result is a compound"
			texp = TopExp.TopExp_Explorer();
			texp.Init(shape,TopAbs.TopAbs_WIRE);
			while ( texp.More() ):
				wire = ts.Wire(texp.Current());
				resultWires.append(WireWrapper(wire));
				texp.Next();
			texp.ReInit();	
		
		return resultWires;
		
	#offset a face, returning the offset shape
	def _offsetFace(self,face,offset ):
		resultWires = [];
		try:

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
			return  bo.Shape();
		except:
			traceback.print_exc(file=sys.stdout);
			print 'error offsetting at offset=',offset		
	
		return None;
		
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
				logging.debug( "Face is at zlevel" + str(zLevel) );
				s.addFace(face);
			texp.Next();
		
		#free memory
		face.Nullify();
		bc.Destroy();
		texp.Clear();
		texp.Destroy();
			
		if not foundFace:
			logging.warn("No faces found after slicing at zLevel " + str(zLevel) + " !. Skipping This layer completely");
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
"""
class Slice:
	def __init__(self):
		logging.debug("Creating New Slice...");
		self.path = "";
		self.faces = [];
		
		#actually these are wirewrappers
		self.fillWires = [];
		self.zLevel=0;
		self.zHeight = 0;
		self.layerNo = 0;
		self.sliceHeight=0;
	
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
		logging.info("Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...");

		gcodewriter = Gcode_Lib.GCode_Generator();
		gcodewriter.verbose = self.verbose;
		gcodewriter.numberFormat = self.numberFormat;
		gcodewriter.start();
		gcodewriter.comment(self.description);
		exportedPaths = 0;
		
		for slice in slices:
			gcodewriter.comment('zLevel' + str(slice.zLevel) );
			logging.info( "zLevel %d, %d paths. " % ( slice.zLevel, len(slice.fillWires)) );
			
			#fw is a wirewrapper
			for fw in slice.fillWires:
				if self.verbose:
					gcodewriter.comment("begin wire");
				logging.info(">>begin wire");
				#print "base wire:",str(fw);				
				sw =fw.getSortedWire();
				#print "sorted wire:",str(sw);				
				gcodewriter.followWire(sw,self.feedRate);
				logging.info(">>end wire");
				
				if self.verbose:
					gcodewriter.comment("end wire");				

				if self.display:
					self.display.showShape(sw.wire);
					
				exportedPaths += 1;

		commands = gcodewriter.getResults();
		f = open(fileName,'w');
		f.write("\n".join(commands));
		f.close()
		

"""
	Read a shape from Step file
"""
def readStepShape(fileName):
	logging.info("Reading STEP file:'" + fileName + "'...");
	stepReader = STEPControl.STEPControl_Reader();
	stepReader.ReadFile(fileName);
	
	numItems = stepReader.NbRootsForTransfer();
	numTranslated = stepReader.TransferRoots();
	logging.info("Read " + str(numTranslated) + " from File.");
	shape = stepReader.OneShape();
	logging.info("Done.");
	return shape;

def readSTLShape(fileName):
	ts = TopoDS.TopoDS();

	logging.info("Reading STL:'" + fileName + "'...");
	#read stl file
	shape = TopoDS.TopoDS_Shape()
	stl_reader = StlAPI.StlAPI_Reader()
	stl_reader.Read(shape,fileName)
	logging.info("Fixing holes and degenerated Meshes...");
	sf = ShapeFix.ShapeFix_Shape(shape);
	sf.Perform();
	fixedShape = sf.Shape();
	logging.info("Making Solid from the Shell...");
	#bb = BRepBuilderAPI.BRepBuilderAPI_MakeSolid(ts.Shell(fixedShape));
	#bb.Build();
	bb = ShapeFix.ShapeFix_Solid();
	return bb.SolidFromShell(ts.Shell(fixedShape));
	logging.info("Done.");
	return bb.Solid();



def printUsage():
	print """
		Usage: OccSliceLib <inputfile: STL or STEP> 
			- inputfile [required] is an STL or STEP file, ending in .stl, .stp, or .step.
		
		Creates an SVG output file compatible with skeinforge	, in same directory as inputfile
	"""
def main(filename):
	global mainDisplay;
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
	
	analyzer = SolidAnalyzer(theSolid);	
	shape = analyzer.translateToPositiveSpace();

	frame.showShape(shape);	
	frame.Show(True)			

	#slicing options: use defaults
	options = SliceOptions();
	options.numSlices=5;
	options.numShells=10;
	options.resolution=0.3;
	options.inFillAngle=15;
	options.inFillSpacing=3;
	
	#slice it
	sliceSet = Slicer(shape,options);
	sliceSet.display = sliceFrame;	
	sliceSet.execute()
	
	#export to gcode
	gexp = GcodeExporter();
	gexp.description="Reprap Test";
	gexp.display = gcodeFrame;
	#gexp.export(sliceSet, outFileName);
	gexp.verbose = False;
	
	app.SetTopWindow(frame)
	app.MainLoop() 

	
if __name__=='__main__':
	nargs = len(sys.argv);

	if nargs > 1:
		filename = sys.argv[1];
		main(filename);
	else:
		printUsage();
		