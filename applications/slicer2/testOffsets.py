import time

from OCC import TopAbs,TopExp
from OCC.Utils.Topology import Topo

import OCCUtil,Util,TestObjects

from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()

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

if __name__=='__main__':
    

    #make a test wire

    startWire = TestObjects.makeOffsetTestWire();

    l = time.clock();    
    #startWire = TestObjects.makeSquareWire();
    numOffsets = makeOffsets(startWire,d=True)
    print "Elapsed: %0.3f, %d offsets" % ((time.clock() - l ),numOffsets);
        
    start_display();