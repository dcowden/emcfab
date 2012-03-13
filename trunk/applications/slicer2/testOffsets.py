import time

from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp,Approx,BRepLib
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
from OCC import BRepExtrema,TColgp,BRepFill
from OCC import ShapeAnalysis
from OCC import ShapeFix,ShapeExtend
from OCC.Utils.Topology import Topo
import OCC.Utils.Topology

import OCCUtil,Util,TestObjects

from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()

brt = BRepTools.BRepTools();
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();

def makeOffsets(wire,d=True):

    numOffsets = 0;

    if d: display.DisplayColoredShape(startWire,'YELLOW');
    STEP = 0.5;
    for offset in Util.frange6(0.5,4.0,STEP):
        #print "offsetting by %0.3f" % offset
        o = OCCUtil.offsetWire(startWire,offset*(-1.0));
        numOffsets+=1;
        if d: display.DisplayColoredShape(o, 'RED');
        
        o2 = OCCUtil.offsetWire(startWire,offset*(-1.0) + (STEP/2.0) );
        numOffsets+=1;
        #create a naive centerline  by setting in( which could conflict with others )
        #if d: display.DisplayColoredShape(o2,'GREEN');
        
        #now offset back out to create centerline. if the result is a compound, then we must offset each member wire
        if o.ShapeType() == TopAbs.TopAbs_COMPOUND:
            t = Topo(o);
            for w in t.wires():
                w2 = OCCUtil.offsetWire(w,STEP*(0.5));
                numOffsets+=1;
                if d: display.DisplayColoredShape(w2, 'BLUE');
        else: #wire
            o2 = OCCUtil.offsetWire(OCCUtil.cast(o),STEP*(0.5));
            numOffsets+=1;
            if d: display.DisplayColoredShape(o2, 'WHITE');

    return numOffsets;

def testLineDirection():
    p1 = gp.gp_Pnt(0,0,0);
    p2 = gp.gp_Pnt(1,0,0);
    p3 = gp.gp_Pnt(-1,0,0);
    e1 = OCCUtil.edgeFromTwoPoints(p1,p2);
    e2 = OCCUtil.edgeFromTwoPoints(p1,p3);
    
    (hCurve1, startp1, endp1) = brepTool.Curve(e1);
    (hCurve2, startp2, endp2) = brepTool.Curve(e2);
    curve1 = hCurve1.GetObject();
    
    q1 = gp.gp_Pnt();
    v1 = gp.gp_Vec();
    q2 = gp.gp_Pnt();
    v2 = gp.gp_Vec();  
    curve1 = GeomAdaptor.GeomAdaptor_Curve(hCurve1);
    curve2 = GeomAdaptor.GeomAdaptor_Curve(hCurve2);
    
    curve1.D1(endp1,q1,v1);
    curve2.D1(endp2,q2,v2);
    
    print v2.Angle(v1);
    print v1.Magnitude(),v1.X(), v1.Y(), v1.Z();
    print v2.Magnitude(),v2.X(), v2.Y(), v2.Z();
    
def testOffsetReferences():

    #f = TestObjects.makeHeartFace();
    #f must be a face with one outer and one inner wire
    f = TestObjects.makeSquareWithRoundHole();
    
    wires = OCCUtil.wireListFromFace(f);
    outer = wires[0];
    inner = wires[1];    
    display.DisplayColoredShape(outer,'GREEN');
    display.DisplayColoredShape(inner,'WHITE');


    #add wires to offset.
    bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
    bo.AddWire(outer);
    bo.AddWire(inner);
        
    bo.Perform(-0.2,0.0);  #do an offset

    shape = bo.Shape();
    for w in Topo(shape).wires():
        display.DisplayColoredShape(OCCUtil.cast(shape),'YELLOW');

    for e in Topo(outer).edges():
        print "Outer Edge %d has %d generated shapes" % ( e.__hash__(), len(OCCUtil.listFromTopToolsListOfShape(bo.Generated(e) ))  );   

    for e in Topo(inner).edges():
        print "Inner Edge %d has %d generated shapes" % ( e.__hash__(), len(OCCUtil.listFromTopToolsListOfShape(bo.Generated(e) ))  );   
    
    display.FitAll();

def testOffSets():
   l = time.clock();    
    #startWire = TestObjects.makeSquareWire();
    #numOffsets = makeOffsets(startWire,d=True)
    #print "Elapsed: %0.3f, %d offsets" % ((time.clock() - l ),numOffsets);
                
if __name__=='__main__':
    
    testLineDirection();
    #testOffsetReferences();
    
    
    start_display();