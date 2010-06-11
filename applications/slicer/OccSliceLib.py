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

			3)Copy OccSliceLib(this script) into a directory of your choice.
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
import hashlib


import math
import itertools

from OCC import STEPControl,TopoDS, TopExp, TopAbs,BRep,gp,GeomAbs,GeomAPI,Message,ShapeExtend,TopTools
from OCC import BRepBuilderAPI,BRepTools,BRepAlgo,BRepBndLib,Bnd,StlAPI,BRepAlgoAPI,BRepAdaptor
from OCC import BRepLProp,BRepGProp,BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepExtrema
from OCC import BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor,IntCurvesFace,Approx,BRepLib

from OCC.Display.wxDisplay import wxViewer3d
from OCC.Utils.Topology import Topo
from OCC.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.ShapeAnalysis import ShapeAnalysis_WireOrder
from OCC.ShapeFix import ShapeFix_Wire
from  OCC.Utils import Topology 
from OCC import ShapeFix
from OCC import BRepBuilderAPI
from OCC import TopTools
from OCC import GProp
from OCC import BRepGProp
from OCC import VrmlAPI
from OCC import gce
#my libraries
#import Gcode_Lib
from Wrappers import *
import hatchLib
import TestDisplay
import cProfile
import pstats
import GcodeExporter
import Wrappers
import SvgExporter
from  SlicerConfig import *

log = logging.getLogger('slicer');


# todo list:
#   Improve performance
#      	2d instead of 3d
#  		section instead of cut
#		better overlapping checks
#		compute bounding box just once
#	thin feature checks
#   instersection checking
#   short edge detection ( remove dumb little edges )
#   stop using memory by generating as we go
#


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

TOLERANCE = 0.0005 # a good value that is precise enough for both MM and IN
UNITS_MM = "mm";
UNITS_IN = "in";
UNITS_UNKNOWN = "units";

def userMessage(msg):
	"message intended for the user. Separate from Logging, which should be only for debugging"
	print "[OccSlicer]:" + msg;

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
			allPoints.extend(pointsFromWire(w,resolution*3));
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
		Compute a set of sane slicing options
	"""
	def defaultSlicingOptions(self):
		options = SlicerOptions();
		
		options.setDefaults( self.guessUnitOfMeasure());
		
		#assume that by default we'll translate to positive space
		options.zMin = 0;
		options.zMax = self.zDim;
		options.translateToPositiveSpace = True;
		
		return options;
		
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
			return SolidAnalyzer(bt.Shape());
		else:
			log.debug("Translation is not required. Returning existing shape");
			return self;
	
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
		
		return UNITS_MM;	

		
"""
	a set of slices that together make a part
"""
class Slicer:
	def __init__(self,analyzer,options):
		"initialize the object"
		self.slices=[]
		self.shape = analyzer.shape;
		self.analyzer = analyzer;
		self.hatchReversed = False;	
		self.saveSliceFaces = True;				
		self.FIRST_LAYER_OFFSET =TOLERANCE*1.5;
		self.options = options;	
			
	def execute(self):
		"slices the object with the settings supplied in the options property"

		if self.options.translateToPositiveSpace:
			userMessage("Translating Object to Positive Space...");
			self.analyzer = self.analyzer.translateToPositiveSpace();		

		shapeToSlice = self.analyzer.shape;
		t = Timer();		
		userMessage("Slicing Started.");
		
		reportInterval = round(self.options.zMax /10);
		
		#
		#make slices
		#
		zLevel = self.options.zMin + self.FIRST_LAYER_OFFSET;
		sliceNumber = 1;
						
		t2 = Timer();
		while zLevel < self.options.zMax:
			#userMessage("Slicing Z= %0.3f ..." % zLevel );
			log.info( "Creating Slice %0d, z=%0.3f " % ( sliceNumber,zLevel));
			slice = self._makeSlice(shapeToSlice,zLevel);
			
			slice.layerNo = sliceNumber;
			

			
			if slice != None:		
				self.slices.append(slice);
				if slice.fillWires.Length() > 0 and self.options.useSliceFactoring:
					log.info("This slice is a duplicate of another one. We are done!");					
				elif self.options.filling.enabled:
					self._fillSlice(slice);
				
			zLevel += self.options.layerHeight;
			sliceNumber += 1;
			self.hatchReversed = not self.hatchReversed;
			#TestDisplay.display.eraseAll();
			#showSlice(slice);
			#time.sleep(2);
			
			#userMessage("Slice z=%03.f : %d paths" % (zLevel,  len(slice.fillWires) + len(slice.fillEdges) ));
			#compute an estimate of time remaining every 10 or so slices
			if reportInterval > 0 and sliceNumber % reportInterval == 0:
				pc = ( zLevel - self.options.zMin )/   (self.options.zMax - self.options.zMin) * 100;
				userMessage("%0.0f %% complete." % (pc) );			

		userMessage("Slicing Complete: " + t.finishedString() );
		userMessage("Throughput: %0.3f slices/sec" % (sliceNumber/t.elapsed() ) );
			
	#fills a slice with the appropriate toolpaths.
	#returns: void
	#a number of wires are added to the slice as paths.
	def _fillSlice(self,slice):
	
		log.info("Filling Slice at zLevel %0.3f" % slice.zLevel);
		log.info("There are %d faces" % slice.faces.Length() );
		
		for f in hSeqIterator(slice.faces):
			log.info("Filling Face..");
			#TestDisplay.display.eraseAll();
			#time.sleep(3);			
			#TestDisplay.display.showShape(f);
			#time.sleep(3);
			numShells = 0;
			t3=Timer();
			shells = [];
			
			#attempt to create the number of shells requested by the user, plus one additional for infill
			#regardless, the last successfully created offset is used for hatch infill
			lastOffset = None;

			#compute one offset for the outer boundary. This is required.If we cannot
			#compute it, we should throw an error
			currentOffset =  (-1) * self.options.nozzleDiameter / 2;
			lastOffset = self._offsetFace(f, currentOffset);

			if lastOffset:
				shells.append(lastOffset);
			else:
				log.error("Cannot compute a single offset. Filling Failed");
				return;

			infillSpacing = self.options.filling.fillWidth;
			numExtraShells = self.options.filling.numExtraShells;
			
			#compute one more offset for filling. If this cannot be computed, again,
			#return
			currentOffset -= infillSpacing;
			lastOffset = self._offsetFace(f, currentOffset);
			if lastOffset:
				shells.append(lastOffset);
			else:
				log.error("Cannot compute a second offset for filling.");
				return;
			
			numShells = 0;
			
			for i in range(2,numExtraShells):
				currentOffset -= infillSpacing;

				try:
					newOffset = self._offsetFace(f,currentOffset);
					if newOffset and self.options.filling.checkFillInterference:					
						r = checkMinimumDistanceForOffset(newOffset,self.options.nozzleDiameter);						
						if not r:
							log.warn("Shell is too close to other shells.");
							break;
							
					shells.append(newOffset);
					numShells+=1;
				except Exception as e:
					traceback.print_exc(file=sys.stdout);
					log.warn(e.args);
					#raise e;

			
			#completed
			if numShells < numExtraShells:
				userMessage(("Could only create %d of the requested %d shells:") % (numShells, numExtraShells ));
			
			#the last shell is for hatch rather than infill.
			#if it was possible to create the last shell, that means it is large enough to fill
			lastShell = shells.pop();
			
			#add shells to the slice
			for s in shells:
				#TestDisplay.display.showShape(s);
				seq =  makeWiresFromOffsetShape(s);
				#for w in hSeqIterator(seq):
				#	ww = Wrappers.Wire(w);
				#	ww.assertHeadToTail();
				#	log.info(str(ww));
				
				for w in hSeqIterator(seq):
					slice.fillWires.Append(w);
			
			#use last shell for hatching
			#debugShape(lastShell);
			h = hatchLib.Hatcher(
					listFromHSequenceOfShape(makeWiresFromOffsetShape(lastShell)),
					slice.zLevel,self.options.filling.fillWidth,
					self.hatchReversed ,
					[ self.analyzer.xMin,self.analyzer.yMin, self.analyzer.xMax, self.analyzer.yMax]);
			
		
			h.hatch();
			#TestDisplay.display.eraseAll();
			for e in h.edges():
				#TestDisplay.display.showShape(e);
				#time.sleep(.1);
				slice.fillEdges.Append(e);
		
		
		log.warn("Filling Complete, Created %d paths." % slice.fillWires.Length()  );


		
	#offset a face, returning the offset shape
	def _offsetFace(self,face,offset ):
		ow = brt.OuterWire(face);
		bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
		bo.AddWire(ow);
		
		
		for w in Topo(face).wires():
			if not w.IsSame(ow):
				#TestDisplay.display.showShape(w);
				bo.AddWire(w);
		
		#print "about to offset by %0.2f" % offset;
		bo.Perform(offset,TOLERANCE);  #this line crashes hard, but only sometimes.
		#print "done offsetting..";
		if  not bo.IsDone():
			raise Exception, "Offset Was Not Successful.";
		else:
			return  bo.Shape();
			
	def _makeSlice(self,shapeToSlice,zLevel):

		s = Slice();

		#used to determine if a slice is identical to others.
		s.hatchDir = self.hatchReversed;
		s.fillWidth = self.options.filling.fillWidth;
		
		#change if layers are variable thickness
		s.sliceHeight = self.options.layerHeight;		
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
		
		foundFace = False;
		for face in Topo(cutShape).faces():
			if self._isAtZLevel(zLevel,face):
				foundFace = True;
				log.debug( "Face is at zlevel" + str(zLevel) );
				s.addFace(face);
				
		if self.options.useSliceFactoring:
			mySum = s.getCheckSum();
			#print 'Slice Created, Checksum is',mySum;
			for otherSlice in self.slices:
				#print "Slice Checksum=",otherSlice.getCheckSum();
				if mySum == otherSlice.getCheckSum():
					log.info("This slice matches another one exactly. using that so we can save time.");
					return otherSlice.copyToZ(zLevel);
				
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
		self.fillWires = TopTools.TopTools_HSequenceOfShape();
		self.fillEdges = TopTools.TopTools_HSequenceOfShape();
		#self.boundaryWires = [];
		self.zLevel=0;
		self.zHeight = 0;
		self.layerNo = 0;
		self.sliceHeight=0;
		self.faces = TopTools.TopTools_HSequenceOfShape();
		self.fillWidth = None;
		self.hatchDir = None;
		self.checkSum = None;
		
	"TODO: Do we really need this?"
	def addFace(self, face ):
		copier = BRepBuilderAPI.BRepBuilderAPI_Copy(face);
		self.faces.Append(ts.Face(copier.Shape()));
		#self.faces.append(face);
		copier.Delete();
	
	def getBounds(self):
		"Get the bounds of all the faces"
		t = Timer();
		box = Bnd.Bnd_Box();
		b = BRepBndLib.BRepBndLib();	

		for f in hSeqIterator(s.faces):
			b.Add(f,box);
			
		bounds = box.Get();
		xMin = bounds[0];
		xMax = bounds[3];
		yMin = bounds[1];
		yMax = bounds[4];
		zMin = bounds[2];
		zMax = bounds[5];
		return [xMin,xMax,yMin,yMax,zMin,zMax];
		
	def copyToZ(self,z):
		"makes a copy of this slice, transformed to the specified z height"
		theCopy = Slice();
		theCopy.zLevel = z;
		theCopy.zHeight = self.zHeight;
		theCopy.sliceHeight = self.sliceHeight;
		theCopy.fillWidth = self.fillWidth;
		theCopy.hatchDir = self.hatchDir;
		theCopy.checkSum = self.checkSum;

		
		#make transformation
		p1 = gp.gp_Pnt(0,0,0);
		p2 = gp.gp_Pnt(0,0,z);
		xform = gp.gp_Trsf();
		xform.SetTranslation(p1,p2);
		bt = BRepBuilderAPI.BRepBuilderAPI_Transform(xform);
		
		#copy all of the faces
		for f in hSeqIterator(self.faces):
			bt.Perform(f,True);
			theCopy.addFace( Wrappers.cast(bt.Shape()));
		
		#copy all of the fillWires
		for w in hSeqIterator(self.fillWires):
			bt.Perform(w,True);
			#TestDisplay.display.showShape(bt.Shape() );
			theCopy.fillWires.Append(Wrappers.cast(bt.Shape()));
		
		#copy all of the fillEdges
		for e in hSeqIterator(self.fillEdges):
			bt.Perform(e,True);
			#TestDisplay.display.showShape(bt.Shape() );
			theCopy.fillEdges.Append(Wrappers.cast(bt.Shape()));
			
		return theCopy;
		
	def getCheckSum(self):
		"produce a checksum that can be used to determine if other wires are practically identical to this one"
		if self.checkSum == None:
		
			PRECISION = 0.1;
			NUMFORMAT = "%0.3f";
			m = hashlib.md5();
			"slice properties that would make them different besides the boundaries"
			m.update(str(self.hatchDir));
			m.update(NUMFORMAT % self.fillWidth);
			m.update(NUMFORMAT % self.faces.Length() );
			gp = GProp.GProp_GProps();
			bg = BRepGProp.BRepGProp();

			for f in hSeqIterator(self.faces):
				bg.SurfaceProperties(f,gp);

			m.update(NUMFORMAT % gp.Mass() );
			m.update(NUMFORMAT % gp.CentreOfMass().X() );
			m.update(NUMFORMAT % gp.CentreOfMass().Y() );
			
			self.checkSum = m.hexdigest();
		
		return self.checkSum;
		
"""
	Read a shape from Step file
"""
def readStepShape(fileName):
	log.info("Reading STEP file:'" + fileName + "'...");
	stepReader = STEPControl.STEPControl_Reader();
	
	
	if not os.path.exists(fileName):
		raise ValueError, "Error: '%s' Does not Exist!" % fileName;
		
	stepReader.ReadFile(fileName);
	
	numItems = stepReader.NbRootsForTransfer();
	numTranslated = stepReader.TransferRoots();
	log.info("Read " + str(numTranslated) + " from File.");
	shape = stepReader.OneShape();
	print "Step file is of type %d" % shape.ShapeType();
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
	sf.SetMaxTolerance(TOLERANCE);

	msgRegistrator = ShapeExtend.ShapeExtend_MsgRegistrator();
	sf.SetMsgRegistrator(msgRegistrator.GetHandle() );
	
	sf.Perform();
	
	log.info("ShapeFix Complete.");
	for i in range(0,18):
		log.info( "ShapeFix Status %d --> %s" % ( i, sf.Status(i) ));
		
	fixedShape = sf.Shape();
	#fixedShape = shape;
	
	#if the resulting shape is a compound, we need to convert
	#each shell to a solid, and then re-create a new compound of solids
	if fixedShape.ShapeType() == TopAbs.TopAbs_COMPOUND:

		log.warn("Shape is a compound. Creating solids for each shell.");
		builder= BRep.BRep_Builder();
		newCompound = TopoDS.TopoDS_Compound();
		#newCompound = TopoDS.TopoDS_CompSolid();
		builder.MakeCompound(newCompound);
		#builder.MakeCompSolid(newCompound);
		for shell in Topo(fixedShape).shells():
			
			solidBuilder = BRepBuilderAPI.BRepBuilderAPI_MakeSolid(shell);
			solid = solidBuilder.Solid();
			sa = SolidAnalyzer(solid);
			print sa.friendlyDimensions();
			builder.Add(newCompound, solid);
		
		#time.sleep(4);
		#Topology.dumpTopology(newCompound);
		return newCompound;  #return temporarily after the first one
	else:
		log.info("Making Solid from the Shell...");

		solidBuilder = BRepBuilderAPI.BRepBuilderAPI_MakeSolid(ts.Shell(fixedShape));
		return solidBuilder.Solid();
	

def printUsage():
	print """
		Usage: OccSliceLib <inputfile: STL or STEP> <configurationfile>
			- inputfile [required] is an STL or STEP file, ending in .stl, .stp, or .step.
			- configurationfile [optional] is a configuation file, see example.cfg
		"""

def showSlices(slicer):
	"show slices on the debug display"
	for s in slicer.slices:
		showSlice(s);

def showSlice(s):
	"show a single slice"
	for w in hSeqIterator(s.fillWires):
		TestDisplay.display.showShape(w);
	
	for e in hSeqIterator(s.fillEdges):
		TestDisplay.display.showShape(e);
	
def main(filename,userOptions):
	t = Timer();
	perfTimes = {};
	
	ok = False;
	userMessage("Loading File...");
	if filename.lower().endswith('stl'):
		theSolid = readSTLShape(filename);
		ok = True;
	
	if filename.lower().endswith('step') or filename.lower().endswith('stp'):
		theSolid = readStepShape(filename);
		ok = True;
	
	if not ok:
		printUsage();
		return;
	perfTimes['loadObject'] = t.elapsed();
	
	rootFileName = filename[ : filename.rfind( '.' ) ]; 

	analyzer = SolidAnalyzer(theSolid);
	
	options = analyzer.defaultSlicingOptions();
	userMessage("Object Loaded: %s " % analyzer.friendlyDimensions() );
	
	userMessage("** Recommended Slicing Options: **");
	print str(options);
	
	#overwrite defaults with user provided settings
	userMessage("** User Selected Slicing Options...");
	options.merge(userOptions);
	print str(options);
	
	#slice it
	try:
				
		slicer = Slicer(analyzer,options);		
		
		userMessage( "*** Beginning Slicing... ");
		t = Timer();
		slicer.execute()
		userMessage("*** Slicing Complete, %0.1f seconds" % t.elapsed() );
		perfTimes['slice'] = t.elapsed();
		sliceTime = t.elapsed();
		
		if options.gcode.enabled:

			#export gcode
			fileName = rootFileName + options.gcode.fileExtension;
			
			ge = GcodeExporter.GcodeExporter(options.gcode);
			t = Timer();
			userMessage("*** Exporting Gcode... ");
			ge.export( fileName, slicer );
			perfTimes['gcodeExport'] = t.elapsed();
			userMessage("*** Gcode Created: %0.3f seconds ***" % t.elapsed()  );
		else:
			userMessage("*** Gcode output is disabled.");
			
		if options.svg.enabled:
			se = SvgExporter.SVGExporter(slicer,options);

			userMessage("*** Creating SVG ***");
			fileName = rootFileName + options.svg.fileExtension;
			t = Timer();			
			se.export(fileName);
			perfTimes['svgExport'] = t.elapsed();
			userMessage("***SVG Created, %0.3f seconds."  % t .elapsed() );
		else:
			userMessage("*** SVG output is disabled.");
		
		
		userMessage("***Processing Complete.***");
		userMessage("*** Performance Summary ***" );
		for k in perfTimes.keys():
			userMessage( "%s : %0.3f sec." % (k,perfTimes[k] ) );
			
		#cProfile.runctx('slicer.execute()', globals(), locals(), filename="slicer.prof")	;					
		#p = pstats.Stats('slicer.prof')
		#p.sort_stats('time')
		#p.print_stats(0.2);
	except:
		#TestDisplay.display.showShapes(sliceSet.slices.pop().faces );
		traceback.print_exc(file=sys.stdout);
		log.critical( 'Slicing Terminated.');
		raise ValueError,"Cannot Slice."
		
	TestDisplay.display.run();
	
if __name__=='__main__':

	###Logging Configuration
	logging.basicConfig(level=logging.WARN,
						format='%(asctime)s [%(funcName)s] %(levelname)s %(message)s',
						stream=sys.stdout)
	nargs = len(sys.argv);

	userOptions = SlicerOptions();
	if nargs > 1:
		filename = sys.argv[1];
	
		if nargs > 2:
			"second arg is configuration file"
			cFile = sys.argv[2];
			if os.path.exists(cFile):
				execfile(cFile);
				userMessage( "Reading Options from file '%s'" % cFile );
			else:
				userMessage ( "Configuration file '%s' not found." % cFile );
				printUsage();
		main(filename,userOptions);
	else:
		printUsage();
		