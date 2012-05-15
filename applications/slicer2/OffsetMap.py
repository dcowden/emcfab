"""

    a class that manages offsets, and tracks 
    relationships between edges
    
    We start with an initial set of wires, 
    and we offset them sequentially inwards or outwards.
    
    The goal of the class is to track references so that given
    an edge, we can find the wire(s) and edges that were created
    from the given edge.  Hopefully this will allow quickly
    drawing edges/wires in the right order
    
    Suppose we start with wire A, then we offset it to wire B, 
    and then we offset wire B to wire C.  each edge in wire A yields
    one or more edges in B, which in turn yields one or more edge
    in wire C.
""" 
from OCC import BRepOffsetAPI,BRepTools, BRep,TopAbs,gp

from OCC.Utils.Topology import Topo
import OCC.Utils.Topology
import OCCUtil,Util,TestObjects,WireJoiner
import time,traceback
import Util
import math
brt = BRepTools.BRepTools();
brepTool = BRep.BRep_Tool();

class OffsetMap:
    def __init__(self,face):
        self.face= face;
        self.originalWires = OCCUtil.wireListFromFace(face);
        self.edgeMap = {};
        self.otherWires = [];
        self.lastWires = self.originalWires;
        self.display = None;
        self.outputWires = [];
    """
        starting with a vertex on the original wire, 
        draw all of the wires in this set exactly once, following them
        in the best order by moving inward.
        
        
        This version is a super-simple one, which assumes that:
           * a nearest vertex search will always identiofy the best wire,
           * any vertex for an adjoining wire will be < pathWidth away
           
        TODO: performance improvements may be possible by using either: 
              * a kdtree to search for points faster than brute force
              * decendants to find the best wire and avoid searching all wires every time
    """
    def followWires (self,startNearPoint,pathWidth):
        
        outputWires = self.outputWires;

        #list wires we must draw. we want to connect them in a sane order
        #make sure to start at a point far outside the face so we start outside!
        toDraw = self.originalWires + self.otherWires;

        #find starting vertex and wire
        #must start on outer wire only
        lastPoint = startNearPoint;
        initialPoint =  OCCUtil.nearestVertex(self.originalWires,startNearPoint);
        lastVector = gp.gp_Vec((gp.gp_Pnt(0,0,0)), startNearPoint);


        #assume initial vector is sideways for now.  At the beginning of a layer, the start vector is not relevant,
        #so anythign will work i think. here we assume the first vector is from the origin
        
        if initialPoint is None:
            raise Exception ("Cannot find initial starting point on outer wire!");

        (currentWire,currentVertex,currentPoint,distance) = OCCUtil.nearestVertex(self.originalWires,startNearPoint);


        while len(toDraw) > 0:

            currentPoint = brepTool.Pnt(currentVertex);    
            toDraw.remove(currentWire);                
                      
            #trim the wire at this point
            #TODO: simplify syntax for this!
            jrequest = WireJoiner.JointRequest( lastPoint,lastVector,pathWidth,currentWire);        
            wj = WireJoiner.WireJoiner(jrequest,pathWidth*(2.0) ); 
            solution = wj.build();            
            
            trimmedWire = solution.wire;
            trimmedEdge = solution.trimmedEdge;
            trimmedPoint = solution.trimmedPoint;
            trimmedVec = solution.trimmedVec;
            
     
            joinEdge = OCCUtil.edgeFromTwoPoints(lastPoint,currentPoint);
            joinWire = OCCUtil.wireFromEdges([joinEdge]);
            outputWires.append(joinWire);
            
            if solution.isJoint:
                #display.DisplayColoredShape(joinWire,'RED');
                pass;                
            else:
                #display.DisplayColoredShape(joinWire,'YELLOW');
                pass;
            outputWires.append(trimmedWire);
            #display.DisplayColoredShape(trimmedWire,'GREEN');
            
            #look for another edge which was created by the trimmedEdge. this is where we should search for a connecting point.
            (nextVertex,nextEdge,nextWire) = self.getNextEdge(trimmedEdge, trimmedPoint);

            if nextVertex:
                #connect edges and chain to the this wire
                currentVertex = nextVertex;
                currentWire = nextWire;
            else:
                #no next edge found. find next closest wire to last point, but don't connect it
                #here we should assume the next move is a rapid and not connect it.
                result = OCCUtil.nearestVertex(toDraw, currentPoint);
                if result:
                    (currentWire,currentVertex,point, distance) = result;
                    lastPoint = trimmedPoint;
                else:
                    #couldnt find next vertex either -- not sure this should ever happen?
                    #TODO this is a total hack, need to fix the loop to avoid this extra check
                    if len(toDraw) > 0:
                        raise Exception("Could not find a next vertex, but there are wires left. how did this happen?")
                    else:
                        break;
            lastPoint = trimmedPoint;
            lastVector = trimmedVec;    
            #time.sleep(1);

    """
        given an edge and a point,
        find the decendant edge, vertex, and wire closest to provided point
        if there is no decendent edge, return None
        TODO: duplicated code to find closed vertex in an edge. duplicated with OCCUtil.nearestVertex
    """
    def getNextEdge(self,edge,point):
        if not self.edgeMap.has_key(edge.__hash__() ):
            return (None,None,None);
              
        nextEdges = self.edgeMap[edge.__hash__() ];
        if nextEdges == None: return None;
        
        sdist = 99999999;
        sv = None;
        sw = None;
        se = None;
        for (ne,nw) in nextEdges:
            for v in Topo(ne).vertices():
                dd = brepTool.Pnt(v).Distance(point);
                if dd < sdist:
                    se = ne;
                    sdist = dd;
                    sv = v;
                    sw = nw;        
        return (sv,se,sw); #return the next edge, wire, and vertex for the given location

    
    """
        simpleoffset
        offset inwards by distance.
    """
    @Util.printTiming
    def offsetOnceSimple(self,distance):
        
        bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();           
        map(bo.AddWire, self.lastWires);
        print "%d wires to offset at step 1, distance = %0.3f" % ( len(self.lastWires),distance);
        bo.Perform(distance*(0.5),0.0);
        result1 = Topo(bo.Shape());

        returnList= [];
        #compound result can be a compound of edges and/or wires. weird weird
        for c in OCCUtil.childShapes(bo.Shape() ):
            display.DisplayColoredShape(c,'BLUE')
            if c.ShapeType() == TopAbs.TopAbs_WIRE:
                returnList.append(c);  #these are actually the wires we want to keep          
            elif c.ShapeType() == TopAbs.TopAbs_EDGE:
                w = OCCUtil.wireFromEdges([c])
                returnList.append(w);
            else:
                print "Warning: compound resulting from offset i am confused about-- not an edge or a wire."
            
        #for each original edge, compute its descendant edges
        #self.edgeMap will contain entries with the original edges, pointing to the generated
        #edges and the corresponding wire:
        #      e1 --> [ (e2, w2 ), (e3 , w3 ) ];
        for w in self.lastWires:
            originalWire = Topo(w);
            for oe in originalWire.edges():
                self.edgeMap[oe.__hash__()] = [];
                
                #find generated values from first transformation
                generatedStep1 = OCCUtil.listFromTopToolsListOfShape(bo.Generated(oe));
                for ne in generatedStep1:
                    #get wire this belongs to this returns a list but how could there ever be more than one?
                    gwires = []
                    for g in result1.wires_from_edge(ne):
                        gwires.append(g);
                    self.edgeMap[oe.__hash__()].append ( (ne,gwires[0]   ));
        
        self.lastWires = returnList;
        self.otherWires.extend(returnList);
        return returnList;
    
    """
        here we offset inwards the specified distance,
        but by first offsetting inwards by distance*2 and then
        offsetting back outwards.
        
        Returns the newly created wires
    """
    def offsetOnce(self,distance):
        
        bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();           
        map(bo.AddWire, self.lastWires);
        print "%d wires to offset at step 1, distance = %0.3f" % ( len(self.lastWires),distance);
        bo.Perform(distance,0.0);
        result1 = Topo(bo.Shape());


        #now offset back outwards
        bo2 = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
        for w in result1.wires():
            bo2.AddWire(w);
        print "Offsetting %0.3f" % ( (-0.5)*distance);
        bo2.Perform((-0.5)*distance, 0.0);
        result2 = Topo(bo2.Shape());


        returnList= [];
        #compound result can be a compound of edges and/or wires. weird weird
        for c in OCCUtil.childShapes(bo2.Shape() ):
            #display.DisplayColoredShape(c,'BLUE')
            if c.ShapeType() == TopAbs.TopAbs_WIRE:
                returnList.append(c);  #these are actually the wires we want to keep
                self.otherWires.append(c);            
            elif c.ShapeType() == TopAbs.TopAbs_EDGE:
                w = OCCUtil.wireFromEdges([c])
                returnList.append(w);
                self.otherWires.append(w);            
            else:
                print "Warning: compound resulting from offset i am confused about-- not an edge or a wire."

        if len(returnList) == 0:   
            return returnList; #do nothing further if the offset computed no curves
        else:
            print "2nd step yielded %d wires" % len(returnList)
            
        #for each original edge, compute its descendant edges
        #self.edgeMap will contain entries with the original edges, pointing to the generated
        #edges and the corresponding wire:
        #      e1 --> [ (e2, w2 ), (e3 , w3 ) ];
        for w in self.lastWires:
            originalWire = Topo(w);
            for oe in originalWire.edges():
                self.edgeMap[oe.__hash__()] = [];
                
                #find generated values from first transformation
                generatedStep1 = OCCUtil.listFromTopToolsListOfShape(bo.Generated(oe));
                for ne in generatedStep1:
                    #find edges generated from that original edge
                    generatedStep2 = OCCUtil.listFromTopToolsListOfShape(bo2.Generated(ne));
                    for ge in generatedStep2:
                        #get wire this belongs to this returns a list but how could there ever be more than one?
                        gwires = []
                        for g in result2.wires_from_edge(ge):
                            gwires.append(g);
                        self.edgeMap[oe.__hash__()].append ( (ge,gwires[0]   ));
        
        self.lastWires = returnList;
        return returnList;


def testOffsetMapBasic():
    #tests whether the offestting works

    f = TestObjects.makeSquareWithRoundHole(); #face that will have odd offsets
    om = OffsetMap(f);    
    om.offsetOnce(-0.2);
    
    display.DisplayColoredShape(om.originalWires,'GREEN');      
    display.DisplayColoredShape(om.otherWires,'RED');
    
def testOffsetMapDecendants():
    #test that we can find the edge, vertex, and wire from a given edge
    
    f = TestObjects.makeSquareWithRoundHole(); #face that will have odd offsets
    om = OffsetMap(f);    
    om.offsetOnce(-0.2);
        
    #display each edge and its decendant edge.
    #each original edge and vertex is checked
    #in each case the result should be the edge that was created from the one in green,
    #and the blue point should be the vertex of that edge closest to the green vertex
    for w in om.originalWires:
        for e in Topo(w).edges():
            for v in Topo(e).vertices():
                display.EraseAll();
                display.DisplayColoredShape(v,'GREEN');
                display.DisplayColoredShape(e,'GREEN');
                (nv,ne,nw) = om.getNextEdge(e,v);
                display.DisplayColoredShape(nw,'YELLOW');
                display.DisplayColoredShape(ne,'RED');
                display.DisplayColoredShape(nv,'BLUE');
                time.sleep(5);


def testBasicFaceOffset2():
    f = TestObjects.makeHeartFaceNoHole();
    om = OffsetMap(f);    
    om.offsetOnceSimple(-0.2);
    om.offsetOnceSimple(-0.2);
    om.offsetOnceSimple(-0.2);
    #om.offsetOnce(-0.2);

@Util.printTiming
def testBasicFaceOffset():
    f = TestObjects.makeHeartFaceNoHole();
    startWires = OCCUtil.wireListFromFace(f);
    display.DisplayColoredShape(startWires,'GREEN');
    cw = startWires;
    for i in range(3):
        w = OCCUtil.offsetWireList(cw,-0.2);
        display.DisplayColoredShape(w,'RED');
        cw = w;

@Util.printTiming        
def testOffsetMapPathWalker():
    #test that we can walk all of the wires in the offset map exactly once by using getNextEdge
    
    #f = TestObjects.makeSquareWithRoundHole(); #face that will have odd offsets
    f = TestObjects.makeHeartFace();
    om = OffsetMap(f);
    #display.DisplayColoredShape(om.originalWires,'GREEN');
    #om.display = display;
    
    #WEIRD BUG-- after one offset, things are good, but after two offsets,
    #the curve is offest back outwards instead of inwards?
    wires1 = om.offsetOnceSimple(-0.1);
    wires2 = om.offsetOnceSimple(-0.1);
    wires3 = om.offsetOnceSimple(-0.1);
    #display.DisplayColoredShape(wires1,'RED');
    #display.DisplayColoredShape(wires2,'RED');
    #display.DisplayColoredShape(wires3,'RED');
    display.DisplayColoredShape(om.originalWires,'GREEN');
    #display.DisplayColoredShape(om.otherWires, 'RED');
    
    ##wires2 = om.offsetOnceSimple(-0.2);
    #display.DisplayColoredShape(wires2,'RED');
    #wires3 = om.offsetOnceSimple(-0.2);
    #display.DisplayColoredShape(wires3,'RED');
    #wires4 = om.offsetOnceSimple(-0.2);  #<-------- breaks!
    #display.DisplayColoredShape(wires4,'RED');
    #CHAINED OFFSETTING IS NOT WORKING---- MUST KEEP THEM TOGETHER AS a face  
    #wires1 = om.offsetOnce(-0.2);
    #display.DisplayColoredShape(wires1,'RED');
    #wires2 = om.offsetOnce(-0.2);
    #display.DisplayColoredShape(wires2,'RED');
     
    #display.DisplayColoredShape(om.originalWires,'GREEN');      
    #display.DisplayColoredShape(om.otherWires,'RED');
    #time.sleep(5);
    #display.EraseAll();
    #om.followWires(gp.gp_Pnt(-2.0,-2.0,0),0.2);
    #display.DisplayColoredShape(om.outputWires,'RED');
    
if __name__=='__main__':
    from OCC.Display.SimpleGui import *
    display, start_display, add_menu, add_function_to_menu = init_display()
    try: 
        #testBasicFaceOffset();  
        testOffsetMapPathWalker();
    except:
        traceback.print_exc();
    start_display();