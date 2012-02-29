
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
from OCC import Voxel
#from OCC.SMESH import *
#from OCC.StdMeshers import *
from OCC.MeshVS import *	

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


from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()

brt = BRepTools.BRepTools();
btool = BRep.BRep_Tool();
ts = TopoDS.TopoDS();
topexp = TopExp.TopExp()
texp = TopExp.TopExp_Explorer();
BRepLProp_CurveTool = BRepLProp.BRepLProp_CurveTool();



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
			
			if slice != None:
				slice.layerNo = sliceNumber;
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
		
			print("computing pixel grid of face...");
			t4=Timer();
			grid = computePixelGrid(f,0.05);
			print("done computing grid. it has %d points" % len(grid) );
			print ("total time %0.3f ms" % ( t4.elapsed()*1000) );
			log.info("Filling Face..");
			#TestDisplay.display.eraseAll();
			#time.sleep(3);			
			TestDisplay.display.showShape(f);
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
			log.info("Offsetting...");
			lastOffset = self._offsetFace(f, currentOffset);
			log.info("Offset complete.");
			if lastOffset:
				shells.append(lastOffset);
			else:
				log.error("Cannot compute a single offset. Filling Failed");
				return;

			infillSpacing = self.options.nozzleDiameter;
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
			tlst = [];
			for e in h.edges():
				tlst.append(e);
				#TestDisplay.display.showShape(e);
				#time.sleep(.1);
				slice.fillEdges.Append(e);
		
			display.DisplayShape(tlst,update=False);
			display.FitAll();
		log.warn("Filling Complete, Created %d paths." % slice.fillWires.Length()  );


		
	#offset a face, returning the offset shape
	def _offsetFace(self,face,offset ):
		ow = brt.OuterWire(face);
		bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
		bo.AddWire(ow);
		
		
		for w in Topo(face).wires():
			if not w.IsSame(ow):
				TestDisplay.display.showShape(w);
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
				TestDisplay.display.showShape(face);
				
				log.debug("Face" + str(face) );

				
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


def showSlices(slicer):
	"show slices on the debug display"
	for s in slicer.slices:
		showSlice(s);

		

		