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
import hatchLib
import TestDisplay
import cProfile
import pstats

###Logging Configuration
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
                    stream=sys.stdout)

log = logging.getLogger('slicer');
log.setLevel(logging.WARN);

					

# todo list:
#   Improve performance
#      	2d instead of 3d
#  		section instead of cut
#		better overlapping checks
#		compute bounding boc just once
#	thin feature checks
#   alternate hatch fill direction
#   stop using memory by generating as we go
#
"""
	##
	##  Performance Improvement Ideas
	## 
    Compute slice boundaries once instead of per layer
	Generate gcode as you go instead of storing in memory
	 
 
"""

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

			#compute one offset for the outer boundary. This is required.If we cannot
			#compute it, we should throw an error
			currentOffset =  (-1) * self.resolution / 2;
			lastOffset = self._offsetFace(f, currentOffset);
			if lastOffset:
				shells.append(lastOffset);
			else:
				log.error("Cannot compute a single offset. Filling Failed");
				return;
			
			#compute one more offset for filling. If this cannot be computed, again,
			#return
			currentOffset -= self.resolution;
			lastOffset = self._offsetFace(f, currentOffset);
			if lastOffset:
				shells.append(lastOffset);
			else:
				log.error("Cannot compute a second offset for filling.");
				return;
			
			numShells = 0;
			for i in range(2,self.options.numExtraShells):
				currentOffset -= self.resolution;

				try:
					#print "Offsetting at zlevel %0.3f" % slice.zLevel
					newOffset = self._offsetFace(f,currentOffset);
					if newOffset:
						r = checkMinimumDistanceForOffset(newOffset,self.options.resolution);						
						if not r:
							log.warn("Shell is too close to other shells.");
							break;
							
					#TestDisplay.display.showShape(newOffset);
					shells.append(newOffset);
					numShells+=1;
				except Exception as e:
					traceback.print_exc(file=sys.stdout);
					log.warn(e.args);
					#raise e;

			
			#completed
			if numShells < self.options.numExtraShells:
				log.warn(("Could only create %d of the requested %d shells:") % (numShells, self.options.numExtraShells ));
			
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
			h = hatchLib.Hatcher(
					listFromHSequenceOfShape(makeWiresFromOffsetShape(lastShell),TopAbs.TopAbs_WIRE),
					slice.zLevel,self.options.inFillSpacing,
					self.options.inFillAngle,
					[ self.analyzer.xMin,self.analyzer.yMin, self.analyzer.xMax, self.analyzer.yMax]);
			
			h.hatch();
			for e in h.edges():
				slice.fillEdges.append(e);
					
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
		global mainDisplay;
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

def showSlices(slicer):
	"show slices on the debug display"
	for s in slicer.slices:
		for w in s.fillWires:
			TestDisplay.display.showShape(w);
		for e in s.fillEdges:
			TestDisplay.display.showShape(e);

def main(filename):

	
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
	
	analyzer = SolidAnalyzer(theSolid);	
	shape = analyzer.translateToPositiveSpace();
		

	#slicing options: use defaults
	options = SliceOptions();
	#options.numSlices=1;
	options.numExtraShells=6;
	options.resolution=0.3;
	options.inFillAngle=45;
	options.inFillSpacing=.3;
	options.zMin=10.8
	options.zMax=12
	
	#slice it
	try:
		sliceSet = Slicer(shape,options);
		sliceSet.execute()
		
		showSlices(sliceSet);
		
		#cProfile.runctx('sliceSet.execute()', globals(), locals(), filename="slicer.prof")	;			
		
		#p = pstats.Stats('slicer.prof')
		#p.sort_stats('time')
		#p.print_stats(0.2);
	except:
		TestDisplay.display.showShapes(sliceSet.slices.pop().faces );
		traceback.print_exc(file=sys.stdout);
		log.critical( 'Slicing Terminated.');	
		
	TestDisplay.display.run();
	
if __name__=='__main__':
	nargs = len(sys.argv);

	if nargs > 1:
		filename = sys.argv[1];
		
		main(filename);
	else:
		printUsage();
		