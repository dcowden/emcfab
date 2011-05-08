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
from  OCC.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape
from  OCC.BRepOffset import *
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
import Wrappers


log = logging.getLogger('slicer');


from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()
display.SetModeWireFrame();

#print(dir(display))
brt = BRepTools.BRepTools();
btool = BRep.BRep_Tool();
ts = TopoDS.TopoDS();
topexp = TopExp.TopExp()
texp = TopExp.TopExp_Explorer();
BRepLProp_CurveTool = BRepLProp.BRepLProp_CurveTool();


TOLERANCE = 0.0005
UNITS_MM = "mm";
UNITS_IN = "in";
UNITS_UNKNOWN = "units";


"""
	A floating point range generator
"""		
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


def fixShape(shape):
	"""
		fixes a shape
	"""
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
	return fixedShape;
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
			builder.Add(newCompound, solid);
		
		#time.sleep(4);
		#Topology.dumpTopology(newCompound);
		return newCompound;  #return temporarily after the first one
	else:
		log.info("Making Solid from the Shell...");

		solidBuilder = BRepBuilderAPI.BRepBuilderAPI_MakeSolid(ts.Shell(fixedShape));
		return solidBuilder.Solid();
	


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

	#puke
	return shape;

def readSTLShape(fileName):


	ts = TopoDS.TopoDS();

	log.info("Reading STL:'" + fileName + "'...");
	#read stl file
	shape = TopoDS.TopoDS_Shape()
	stl_reader = StlAPI.StlAPI_Reader()
	stl_reader.Read(shape,fileName)
	return fixShape(shape);
	
def drillWithHoles(shape):
	"returns a shape with a bunch of spheres removed from it"
	box = Bnd.Bnd_Box();
	b = BRepBndLib.BRepBndLib();
	b.Add(shape,box);
	
	cShape = shape;
	(xMin,yMin,zMin,xMax,yMax,zMax ) = box.Get();

	d = 0.05* ( (xMax - xMin) );
	di = 4.0*d;
	c = 0;
	
	#drill holes in a rectangular grid
	for  x in frange6(xMin,xMax,di):
		for y in frange6 ( yMin,yMax,di):
			for z in frange6(zMin,zMax,di):
				c += 1;
				print "Inter %d " % c;
				#make a sphere
				center = gp.gp_Pnt(x,y,z);
				hole = BRepPrimAPI.BRepPrimAPI_MakeSphere(center,d).Shape();
				cut = BRepAlgoAPI.BRepAlgoAPI_Cut(cShape,hole);
				cut.SetOperation(3); #3 is cut21
				tmp = cut.Shape(); #store the newly cut shape
				if cut.ErrorStatus() != 0:
					print "Error %d cutting" % cut.ErrorStatus()
				else:
					print "Success!"
					cShape = tmp;
	return cShape;

def squareWire(centerPt,w ):
	"makes a square wire with center at the desired point"
	w2 = w/2.0;
	p1 = gp.gp_Pnt(centerPt.X() - w2,centerPt.Y() -w2 , centerPt.Z() )
	p2 = gp.gp_Pnt(centerPt.X() - w2,centerPt.Y() +w2, centerPt.Z() )
	p3 = gp.gp_Pnt(centerPt.X() + w2,centerPt.Y() +w2, centerPt.Z() )
	p4 = gp.gp_Pnt(centerPt.X() + w2,centerPt.Y() -w2, centerPt.Z() )
	e1 = Wrappers.edgeFromTwoPoints(p1,p4);
	e2 = Wrappers.edgeFromTwoPoints(p4,p3);
	e3 = Wrappers.edgeFromTwoPoints(p3,p2);
	e4 = Wrappers.edgeFromTwoPoints(p2,p1);
	return Wrappers.wireFromEdges([e1,e2,e3,e4] );
	
def drillWithHolesFaster(shape):
	"returns a shape with a bunch of spheres removed from it"
	box = Bnd.Bnd_Box();
	b = BRepBndLib.BRepBndLib();
	b.Add(shape,box);
	

	(xMin,yMin,zMin,xMax,yMax,zMax ) = box.Get();

	d = 0.05* ( (xMax - xMin) );
	di = 3.0*d;
	vec = gp.gp_Vec(gp.gp_Pnt(0,0,0),gp.gp_Pnt(0,0,d));
	cp = None;
	compound = TopoDS.TopoDS_Compound();
	builder = BRep.BRep_Builder();
	builder.MakeCompound(compound);
	#drill holes in a rectangular grid
	for  x in frange6(xMin,xMax,di):
		for y in frange6 ( yMin,yMax,di):
			for z in frange6(zMin,zMax,di):
				#make a sphere
				center = gp.gp_Pnt(x,y,z);
				hole = BRepPrimAPI.BRepPrimAPI_MakeSphere(center,d).Shape();
				#lets see if a square hole is faster!
				#w = squareWire(center,d );
				#hb = BRepPrimAPI.BRepPrimAPI_MakePrism(w,vec,False,True);
				#hb.Build();
				#hole = hb.Shape();
				builder.Add(compound,hole);

	display.DisplayShape(compound);
	q = time.clock();		
	cut = BRepAlgoAPI.BRepAlgoAPI_Cut(shape,compound);
	if cut.ErrorStatus() == 0:
		print "Cut Took %0.3f sec." % ( time.clock() - q );
		return cut.Shape();	
	else:
		print "Error Cutting: %d" % cut.ErrorStatus();
		return shape;
	
def offSetShell(shape):
	b = BRepOffsetAPI_MakeOffsetShape(solid,-0.15,0,False,False);
	b.Build();
	return b.Shape();
if __name__=='__main__':
	"do some tests of offset shelling."

	#load solid
	fn = "parts\\RepRapTest.STEP"
	fn = "parts\\simple_2.stp";#doesnt work-- c1 error 
	#fn = "parts\\SimpleTwoBody2.STEP"; #doesnt work: disconnected shell error
	#fn = "parts\\Block.STEP"; #doesnt work, no error , but no object
	#fn = "parts\\simple2.STEP"; #doesnt work-- c1 error
	fn = "parts\\Test.STEP"; #worked
	solid = fixShape(readStepShape(fn));
	display.DisplayShape(solid);
	
	#newshape = drillWithHolesFaster(solid);
	newshape = offSetShell(solid);
	#display.EraseAll();
	display.DisplayShape(newshape);
	print "Done!"
	start_display();
