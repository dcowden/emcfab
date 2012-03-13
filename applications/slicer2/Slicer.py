"""
    Slices a solid object.
    Input: a solid OCC shape ( could be from STEP or STL )
    Output: a collection of layers containing paths. These are typically converted to gcode and/or svg
    
"""
import hashlib
import time

from OCC import TopTools,BRepBndLib,Bnd,gp,BRepBuilderAPI,GProp,BRepGProp,BRepPrimAPI,BRepAlgoAPI
from OCC.Utils.Topology import Topo

import SlicingOptions
import Wrappers
import OCCUtil
from Extruder import *
from Constants import *;

import hatchlib
import Util
import debugdisplay


"""
    Slices a solid,
    
    Still to implement:
        (1) Filling
            (a) implement fill density
            (b) implement alternating fill angle
            (c) join paths
        (2) Output
            (a) SVG output
            (b) Gcode output
            
        (3) Testing: try loading STL
        (4) support
        (5) hexagonal fill ( even though it is slow )
        
    Performance ideas: 
        (0) profiling
        (1) psyco
        (2) go to 2d for filling-- saves performance on computation, and avoids transforms on copying
        (3) remove/optimize close point computation during filling?
        (4) during offsetting, instead of two offsets per shell, try one at the desired offset. If it works, 
            then we know we can inset to that point without checking each time?
            (a) can we speed offsetting in 2D?
        (5) try slicing with a plane instead of halfspace ? ( BRepAlgoAPI_Section)
    
"""
class Slicer:    
    """
        Load the shape, and compute slicing options
    """
    def __init__(self,shape,options):
        
        self.solid = Wrappers.Solid(shape);
        self.slices = []; #a list of slices
        self.sliceMap = {}; #distinct slices. different because some slices are re-used instead of calculated again
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
    @Util.printTiming
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
    #@Util.printTiming
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
                cSlice.addFace(Face(face));
                
        mySum = cSlice.computeFingerPrint();

        #
        #uncomment to enable layer copying.
        #
        #check for identical slices. return matching ones if found.
        if self.sliceMap.has_key(mySum):
            #print "i can copy this layer!"
            return self.sliceMap[mySum].copyToZ(zLevel,layerNo);
                
        
        self.sliceMap[mySum] = cSlice;
        print "This slice has %d faces." % len(cSlice.faces)
        #fill each face in the slice
        for f in cSlice.faces: #Face object

            #how many shells do we need?
            #attempt to create the number of shells requested by the user, plus one additional for infill

            numShells = (int)( self.options.fillingWallThickness / self.extruder.trackWidth()) + 1;
            numShells = max(2,numShells);
            faceWires = OCCUtil.wireListFromFace(f.face);
            
            #regardless, the last successfully created offset is used for hatch infill
            shells = [];
            for i in range(1,numShells):
                #compute offset inwards by one track width               
                offset =  (-1) * i *  self.extruder.trackWidth();                
                innerEdge = OCCUtil.offsetWireList(faceWires,offset);
                
                if len(innerEdge) == 0:
                    #performance: dont offset if there were already no wires
                    continue;
                
                pathCenter = OCCUtil.offsetWireList(innerEdge,self.extruder.trackWidth()/2);
                if len(pathCenter) > 0:
                    shells.append(pathCenter);

            if len(shells) > 1:
                #use last one for filling.
                print "%d shells were available for Hatching" % len(shells)
                lastShell = shells.pop();
                s = self.solid;
                h = hatchlib.Hatcher(lastShell,zLevel,(s.xMin,s.yMin,s.xMax,s.yMax), self.extruder.trackWidth(), cSlice.fillAngle );
                h.hatch();
                ww = h.getWires();
                print "Hatching complete: %d fillWires created" % len(ww )
                f.fillWires = ww;
            else:
                print "WARNING: not filling this layer, too few shells were computable"
            #add shells 
            for s in shells:
                for ss in s:
                    f.shellWires.append(ss);
                
        return cSlice;
    
    def display(self):
        """
            for debugging purposes, display the result
        """                
        for slice in self.slices:            
            for f in slice.faces:
               for w in f.fillWires:
                 debugdisplay.DisplayColoredShape(w, 'BLUE',False);
               for w in f.shellWires:
                 debugdisplay.DisplayColoredShape(w,'GREEN',False);
        debugdisplay.FitAll();
        debugdisplay.start_display();
        
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
        self.faces = [];
        self.layerNo = None;
        self.fillAngle = None;
        self.fingerPrint = None;
        
    def addFace(self, face ):
        #copier = BRepBuilderAPI.BRepBuilderAPI_Copy(face);
        #self.faces.Append(ts.Face(copier.Shape()));
        #self.faces.append(face);
        #copier.Delete();
        self.faces.append( face);

        
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
            m.update(NUMFORMAT % len(self.faces) );
            gp = GProp.GProp_GProps();
            bg = BRepGProp.BRepGProp();
    
            for f in self.faces:
                bg.SurfaceProperties(f.face,gp);
    
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
        for f in self.faces:
            
            bt.Perform(f.face,True);
            newFace = Face(OCCUtil.cast(bt.Shape()));
            
            #copy shells
            for shell in f.shellWires:
                #print shell
                bt.Perform(shell,True);
                newFace.shellWires.append(OCCUtil.cast(bt.Shape()));
                
            #copy fillWires
            for fill in f.fillWires:
                bt.Perform(fill,True);
                newFace.shellWires.append(OCCUtil.cast(bt.Shape())); 
            theCopy.addFace( newFace);
            
  
        return theCopy;        

"""
    A single Face inside of a slice.
    A single slice can have several faces, each of which needs to be shelled and filled
    
"""
class Face:
    def __init__(self,face):
        self.face = face;
        self.shellWires = [];
        self.fillWires = [];


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
         self.type = None;          # fill, perimeter, support, etc

         
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
        
