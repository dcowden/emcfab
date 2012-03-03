"""
    Slices a solid object.
    Input: a solid OCC shape ( could be from STEP or STL )
    Output: a collection of layers containing paths. These are typically converted to gcode and/or svg
    
"""
import hashlib

from OCC import TopTools,BRepBndLib,Bnd,gp,BRepBuilderAPI,GProp,BRepGProp,BRepPrimAPI,BRepAlgoAPI
from OCC.Utils.Topology import Topo

import SlicingOptions
import Wrappers
import OCCUtil
from Extruder import *
from Constants import *;
import time
import hatchlib

from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()

class Slicer:    
    """
        Load the shape, and compute slicing options
    """
    def __init__(self,shape,options):
        
        self.solid = Wrappers.Solid(shape);
        self.slices = [];
        self.options = options;
        
        self.__setDefaultOptions();
        
        self.extruder = Extruder(options);
        
        #note: make sure to fix the shape before translating!
        if options.fixShape:
            self.solid.fixDefects();
            
        if options.translateToPositiveCenter:
            self.solid = self.solid.translateToPositiveCenter(options.tableCenterX, options.tableCenterY);
                        
        #now apply slicing limits, if the user specified them
        #warning, it is confusing if the user specifies these, and specifies translate=true, how do they know where zMin and zMax will be?
        if options.zMin == None:
            options.zMin = self.solid.zMin;
        
        if options.zMax == None:
            options.zMax = self.solid.zMax;
        
    def __setDefaultOptions(self):
        """
            sets defaults that may not have been set by the user.
            This is done here rather than inside of Slicing Defaults because
            in several cases, knowledge of the part and the options is required to do a good job.
        """
        options = self.options;
        errs=options.errors();
        if len(errs) > 0:
            raise Exception("Problem with Options: " + str(errs));
        
        #does user want to translate to positive space?        
        if options.translateToPositiveCenter == None: 
            options.translateToPositiveCenter = True;

        #guess unit of measure
        if options.units == None:
            options.units = self.solid.guessUnitOfMeasure();
            
        #guess sane values for options not supplied
        if options.trackWidth == None:
            options.trackWidth = options.nozzleDiameter * 1.05;
        
        if options.layerHeight == None:
            options.layerHeight = 0.1;
        
        if options.fillingWallThickness == None:
            options.fillingWallThickness = 1.0; #must normalize to units
            
        #TODO: fix this later with smarter computations
        #hack-- we can do better than this later
        options.generateSupports = False;
        options.machineMaxFeedRate = 200;  
        options.fixShape = True;
        options.fillingEnable = True;                 
        options.gcodeNumberFormat = "%0.3f";            
        options.gcodeVerboseComments = True; 
        options.gcodeUseArcs = True; 
        options.gcodeAxisName = "E"; 
        options.gcodeCurveApproxTolerance = 0.001; 
        options.gcodeFileExtension = ".gcode"; 
        options.svgFileExtension = ".svg";    
        
    
    """
        Slices the object, after which slices will contain a list of slices for printing
        A slice is a horizontal section of the part.
        TODO: add 'skeinning'-- more perimeters than fill lines
    """
    def execute(self):
        #at this point we have a solid ready to execute
        
        zLevel = self.options.zMin + TOLERANCE;        
        layerNo = 1;
        fillAngle = 0;

        while zLevel < self.options.zMax:
            print "Slicing At ZLevel=%0.3f" % zLevel;
            #compute the faces for the slice. It may be a transformed
            #copy of another slice, or a new slice
            cSlice = self._computeSlice(zLevel,layerNo,fillAngle);
            self.slices.append(cSlice)    
            zLevel += self.options.layerHeight;
            
    "computes the boundaries for the given slice"    
    def _computeSlice(self,zLevel,layerNo,fillAngle):
        
        cSlice = Slice();
        cSlice.zLevel = zLevel;
        cSlice.layerNo = layerNo;
        cSlice.fillAngle = fillAngle;
        cSlice.thickness = self.options.layerHeight;
                
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
        bc = BRepAlgoAPI.BRepAlgoAPI_Cut(self.solid.shape,halfspace);
        cutShape = bc.Shape();
        
        foundFace = False;
        for face in Topo(cutShape).faces():
            if OCCUtil.isFaceAtZLevel(zLevel,face):
                foundFace = True;
                cSlice.addFace(face);
                
        mySum = cSlice.computeFingerPrint();

        #check for identical slices. return matching ones if found.
        for otherSlice in self.slices:
            if mySum == otherSlice.fingerPrint:
                #this fingerprint is identical to another one, so transform it instead of computing again
                return otherSlice.copyToZ(zLevel,layerNo);
        
        #else, continue to fill this one         
        
        #fill each face in the slice
        for f in OCCUtil.hSeqIterator(cSlice.faces):
        
            #how many shells do we need?
            #attempt to create the number of shells requested by the user, plus one additional for infill

            numShells = (int)( self.options.fillingWallThickness / self.extruder.trackWidth()) + 1;
            numShells = max(2,numShells);
            
            faceWires = OCCUtil.wireListFromFace(f);
            
            #regardless, the last successfully created offset is used for hatch infill
            lastPath = None;
            for i in range(1,numShells):
                print "computing shell..."
                #compute offset inwards by one track width               
                offset =  (-1) * i *  self.extruder.trackWidth();
                
                innerEdge = OCCUtil.offsetWireList(faceWires,offset);
                
                #display.DisplayShape(innerEdge);
                #time.sleep(1.0);
                #now offset back outwards 1/2 track width. this creates a nice smooth path
                pathCenter = OCCUtil.offsetWireList(innerEdge,self.extruder.trackWidth()/2);
                #display.DisplayShape(pathCenter)
                #display.EraseAll();
                               
                for w in pathCenter:
                    cSlice.fillWires.Append(w);

                lastPath = pathCenter;
                    
            #ok at this point, the last path represents the last successfully computed shell, which 
            #should be the basis for filling
            #opportunity for improvement: this bounds can and should be different for each layer.
            #for lines it is cheaper to compute once for the whole slice object. but for
            #hexagons it is cheaper to compute them only for the area needed.
            
            #ireList,zLevel,bounds,spacing,fillAngle
            s = self.solid;
            h = Hatcher(lastPath,cSlice.zLevel,(s.xMin,s.yMin,s.xMax,s.yMax), self.extruder.trackWidth(), cSlice.fillAngle );
            h.hatch();
            
        return cSlice;
    
    def display(self):
        """
            for debugging purposes, display the result
        """        

        
        for slice in self.slices:
            
            for f in OCCUtil.hSeqIterator(slice.faces):
            #    display.DisplayColoredShape(f,'WHITE');
               for w in OCCUtil.hSeqIterator(slice.fillWires):
                 display.DisplayColoredShape(w, 'BLUE');
        
        start_display();
        
class Slice:
    """
        a slice of a solid object, which needs to be printed
            
        Input: 
            zLevel: float
            thickness: float
            fcase: TopTools.TopTools_HSequenceOfShape
            layerNo: int
            fillAngle: int         
        
        sequence:
            create slice from faces 
                at this point, bounds and checksum are available
            if desired, copy to a new slice, else
            call fill()
    """
    def __init__(self):
        self.zLevel = None;
        self.thickness = None;               
        self.faces = TopTools.TopTools_HSequenceOfShape();
        self.layerNo = None;
        self.fillWires = TopTools.TopTools_HSequenceOfShape();
        self.fillAngle = None;
        self.fingerPrint = None;
        
    def addFace(self, face ):
        #copier = BRepBuilderAPI.BRepBuilderAPI_Copy(face);
        #self.faces.Append(ts.Face(copier.Shape()));
        #self.faces.append(face);
        #copier.Delete();
        self.faces.Append(face);
        
    def computeBounds(self):
        "Get the bounds of all the faces"
        if self.bounds == None:
            box = Bnd.Bnd_Box();
            b = BRepBndLib.BRepBndLib();    
    
            for f in OCCUtil.hSeqIterator(s.faces):
                b.Add(f,box);
                
            bounds = box.Get();
            xMin = bounds[0];
            xMax = bounds[3];
            yMin = bounds[1];
            yMax = bounds[4];
            zMin = bounds[2];
            zMax = bounds[5];
            self.bounds =  [xMin,xMax,yMin,yMax,zMin,zMax];
        return self.bounds;
    
    def computeFingerPrint(self):
        """
            computes a checksum of a slice.
            If two slices are identical, then one layer can simly be copied to the next layer
        """
        if self.fingerPrint == None:
            PRECISION = 0.1;
            NUMFORMAT = "%0.3f";
            m = hashlib.md5();
            "slice properties that would make them different besides the boundaries"
            m.update(str(self.fillAngle));
            m.update(NUMFORMAT % self.thickness);
            m.update(NUMFORMAT % self.faces.Length() );
            gp = GProp.GProp_GProps();
            bg = BRepGProp.BRepGProp();
    
            for f in OCCUtil.hSeqIterator(self.faces):
                bg.SurfaceProperties(f,gp);
    
            m.update(NUMFORMAT % gp.Mass() );
            m.update(NUMFORMAT % gp.CentreOfMass().X() );
            m.update(NUMFORMAT % gp.CentreOfMass().Y() );
            self.fingerPrint = m.hexdigest();
    
        return self.fingerPrint;
    
    def copyToZ(self,z,layerNo):
        "makes a copy of this slice, transformed to the specified z height"

        theCopy =  Slice();
        theCopy.zLevel = z;
        theCopy.fillAngle = self.fillAngle;
        theCopy.layerNo = layerNo;
        theCopy.thickness = self.thickness;     
                
        #make transformation
        p1 = gp.gp_Pnt(0,0,0);
        p2 = gp.gp_Pnt(0,0,z);
        xform = gp.gp_Trsf();
        xform.SetTranslation(p1,p2);
        bt = BRepBuilderAPI.BRepBuilderAPI_Transform(xform);
        
        #copy all of the faces
        for f in OCCUtil.hSeqIterator(self.faces):
            bt.Perform(f,True);
            theCopy.addFace( OCCUtil.cast(bt.Shape()));
        
        #copy all of the fillWires
        for w in OCCUtil.hSeqIterator(self.fillWires):
            bt.Perform(w,True);
            #TestDisplay.display.showShape(bt.Shape() );
            theCopy.fillWires.Append(OCCUtil.cast(bt.Shape()));
            
  
        return theCopy;        

class ExtrusionPath:
    """
        A single path that should be drawn. Does not include travel paths
        
        NOTE: paths should not include Z level information-- this allows copying paths
        from one layer to another without adjustments required
    """ 
    def __init__(self):
         self.startPoint = None;
         self.endPoint = None;
         self.width = None;         #the track width of this path      
         self.path = None;          #either an edge or a wire
         self.speed = None;         #the speed of the move in the x-y plane
         self.extrudate = None;     #amount of movement for the extruder axis
         self.role = None;          # fill, perimeter, support, etc

         
class ExtrusionPathChain:
    """
        a list of paths that should be drawn in the specified order

        NOTE: paths should not include Z level information-- this allows copying paths
        from one layer to another without adjustments required        
    """
    
    def __init__(self):
        self.startPoint = None;
        self.endPoint = None;
        self.paths = [];            #each entry is an ExtrusionPath
        
