#pure python imports
import time,os,sys,string;
import itertools

# OCC imports
from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp,Approx,BRepLib,Bnd,BRepBndLib
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
from OCC import BRepExtrema,TColgp
from OCC import ShapeAnalysis
from OCC.Utils import Topo
from OCC import ShapeFix,ShapeExtend

# project files
import TestObjects
import OCCUtil

brepTool = BRep.BRep_Tool();

"""

    Smart Wire Builder..
    accepts edges, and then uses WireOrder to connect them together into the best possible
    edges

"""
class WireBuilder:
    def __init__(self):
        self.edges = [];
        self.fixer = ShapeFix.ShapeFix_Wire();
        self.wireData = ShapeExtend.ShapeExtend_WireData();

    def add(self,edge):
        self.edges.append(edge)
        self.wireData.Add(edge);

    "returns a list of wires, representing the fewest possible wires to connect the edges"    
    def wire(self):
        #this method will only work when exactly one wire is expected.
        print "Ordering Wires. We have %d Edges" % len(self.edges)
        self.fixer.Load(self.wireData.GetHandle());
        self.fixer.SetClosedWireMode(True)
        self.fixer.FixReorder();
        self.fixer.FixConnected();
        return self.fixer.Wire();
        
"""
    computes distance from provided vertex to the closest place on the provided wire
"""
def distToPoint( wire, vertex):
    brp = BRepExtrema.BRepExtrema_DistShapeShape();
    brp.LoadS1(vertex);
    brp.LoadS2(wire);
    
    result = brp.Perform();
    if result:
        if brp.NbSolution() > 1:
            print "weird, more than one solution"
        
        point = brp.PointOnShape2(1);
        #otherwise, figure out what kind of location we have.
        if brp.SupportTypeShape2(1) == BRepExtrema.BRepExtrema_IsOnEdge:
            #closest point is a point along an edge
            edge = OCCUtil.cast(brp.SupportOnShape2(1));
            parameter = brp.ParOnEdgeS2(1);
            #print "Closest point is on an edge"
        else:
            #closest point is a point on a vertex
            vertex = OCCUtil.cast(brp.SupportOnShape2(1));
            #print "closest point is on vertex"
        
        return point;

"""
    makes an wire path based on the provided wire,
    including moving from the provided startPoint to begin the move
    We assume that the wire is a closed wire.
    
    steps include trimming the wire to account for track width,
    and trimming to account for acute angles
"""
def makeExtrusionWire( shellWire, startPoint, trackWidth):
    
    topoWire = Topo(shellWire);
    #compute closest point on the wire
    brp = BRepExtrema.BRepExtrema_DistShapeShape();
    brp.LoadS1(OCCUtil.make_vertex(startPoint));
    brp.LoadS2(shellWire);
    
    result = brp.Perform();    
    p1 = brp.PointOnShape2(1);
    wb = WireBuilder();

    #make an edge from start point to located point.
    #wb.add ( OCCUtil.edgeFromTwoPoints(startPoint, p1 ) );
    dist = p1.Distance(p2)
    
    
    if brp.SupportTypeShape2(1) == BRepExtrema.BRepExtrema_IsOnEdge:
  
        #closest point is a point along an edge
        interSectingEdge = OCCUtil.cast(brp.SupportOnShape2(1));
        p = brp.ParOnEdgeS2(1);
    
        #compute parameter along one curve
        #break the edge into two new edges. account for a split distance between them.
        (e1,e2)= OCCUtil.splitEdge(interSectingEdge,p);
        
         
        wb.add(e1);
                
        #add second one shortened, on the end near the vertex
        wb.add ( OCCUtil.shortenEdge(e2,p1,trackWidth)); #hack, must find parameter closest to this end

        #now add all of the other edges in the wire except the original one that we split
        for e in topoWire.edges():
            if not e.IsSame(interSectingEdge):
                wb.add(e);

    else:
        #closest point is a point on a vertex, here we'll shorten one edge
        #
        vertex = OCCUtil.cast(brp.SupportOnShape2(1));        
        edges = [];
        for e in  topoWire.edges_from_vertex(vertex):
            edges.append(e);
            
        #shorten one, leave other intact
        #try to handle case where vertex is at end of a wire ( ie a non-closed wire )
        e1 = edges[0];
        wb.add( edges[0]);
        e2 = None;
        if len(edges) > 1:
            e2 = edges[1];            
            e3 = OCCUtil.shortenEdge(e2,p1,trackWidth); #hack-- find edges closest to this end
            #display.DisplayColoredShape(e3,'BLUE')
            wb.add ( e3); 
        
        for e in topoWire.edges():
            if e.IsSame(e1): continue;                
            if e2 and e.IsSame(e2): continue;            
            wb.add ( e );
            
    return wb.wire();


if __name__=='__main__':
    
    from OCC.Display.SimpleGui import *
    display, start_display, add_menu, add_function_to_menu = init_display()
    
    heart = TestObjects.makeHeartWire();
    
    #p1 = gp.gp_Pnt(0.0,-0.5,0.0);
    p1 = gp.gp_Pnt(2.1,1.5,0.0);
    v1 = OCCUtil.make_vertex ( gp.gp_Pnt(2.0,3.1,0.0));
    #t = time.clock();
    #for i in range(1000):
    #    p2 = distToPoint(heart,v1 );    
    #print "dist time %0.3f" % ( time.clock() - t ); #about 1.6ms per call for a simple heart wire . not too shabby
    
    #v2 = OCCUtil.make_vertex ( p2 );
    #display.DisplayShape([heart,v1,v2]);
    
    
    #test splitting a wire
    newWire= makeExtrusionWire(heart,p1,0.25);
    display.DisplayColoredShape(heart, 'WHITE')
 
    display.DisplayColoredShape(newWire, 'GREEN');
    print "Total Edges %d" % Topo(newWire).number_of_edges()
        
    start_display();