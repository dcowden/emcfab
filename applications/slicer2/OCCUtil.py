"""
    Utility functions related to OCC
"""
import time,os,sys,string;

from OCC import BRep,gp,GeomAbs,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp,Approx,BRepLib
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor

from OCC.Utils.Topology import Topo

from Constants import *;

brt = BRepTools.BRepTools();
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();

import itertools


def pnt(x,y):
    return gp.gp_Pnt(x,y,0);

def make_vertex(pnt):
    "make a vertex from a single point"
    s = BRepBuilderAPI.BRepBuilderAPI_MakeVertex(pnt);
    s.Build();
    return s.Shape();

def listFromHSequenceOfShape(seq):
    "return a list from a sequence o shape"
    newList = [];
    for i in range(1,seq.Length()+1):
        newList.append(cast(seq.Value(i)));
    return newList;

def hSeqIterator(hSeq):
    "sane iteration over hsequenceof shape"
    for i in range(1,hSeq.Length()+1):
        yield cast(hSeq.Value(i));
        
def extendhSeq(hSeqTo, hSeqFrom):
    "append elements in from to to"
    for e in hSeqIterator(hSeqFrom):
        hSeqTo.Append(e);

def edgeFromTwoPoints(p1,p2):
    "make a linear edge from two points "
    builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2);
    builder.Build();
    if builder.IsDone():
        return builder.Edge();
    else:
        return None;
        
def findParameterOnCurve(point,handleCurve,tolerance):
    "return the closest parameter on the curve for the provided point"
    "returns none if a point is not within tolerance"
    #TODO: Extrema.Extrema_ExtPC will project a point onto an Adaptor_3d Curve
    #so we do not have to approximate with a bspline
    #BrepAdaptor.BrepAdaptor_CompCurve.Trim() will return a trimmed curve between two parameters
    #E1CLib can project points on elementary curves (lines and conics )
    gp = GeomAPI.GeomAPI_ProjectPointOnCurve(point,handleCurve);
    gp.Perform(point);
    if gp.NbPoints()>0:
        #log.debug( "Projection Success!" );
        return [point, gp.LowerDistanceParameter(),gp.LowerDistance()];
    else:
        #log.error( "Projection Failed.");
        return None;

def approximatedWire(wire):
    "returns a bezier approximation of the specified wire as an edge"
    #TODO: this is also duplicated in hexagonlib, and should be factored here.
    
    #make a parameterized approximation of the wire
    adaptor = BRepAdaptor.BRepAdaptor_CompCurve (wire);
    curve = BRepAdaptor.BRepAdaptor_HCompCurve(adaptor);
    curveHandle = curve.GetHandle();
    
    #approximate the curve using a tolerance
    approx = Approx.Approx_Curve3d(curveHandle,0.01,GeomAbs.GeomAbs_C2,1000,8);
    if approx.IsDone() and  approx.HasResult():
        # have the result
        anApproximatedCurve=approx.Curve();

        builder =  BRepLib.BRepLib_MakeEdge(anApproximatedCurve);
        e = builder.Edge();
        return Wrappers.wireFromEdges([e]);        

def isEdgeLocalMinimum(edge,param,point):
    """
        checks to see if the location on an edge is a local minimum or max.
        see notes below: this must be changed if the coordinate system is rotated--
        
        TODO: fix assumption that scanlines are in the x direction
    """
    ALITTLEBIT=0.01;
    
    p1 = pointAtParameter(edge,param-ALITTLEBIT);
    p2 = pointAtParameter(edge,param+ALITTLEBIT);
    
    d1 = p1.Y() - point.Y();
    d2 = p2.Y() - point.Y();
    
    if d1*d2 < 0:
        #edge counts as an intersection
        return False;
    else:
        #TestDisplay.display.showShape(make_vertex(point));
        return True;
    
    
    
def isLocalMinimum ( wire, vertex ):
    """
        The vertex is assumed to exist on the wire.
        
        The method determines if the vertex is situated such that the 
        location is a local minimum or maxiumum on the boundary.
        
        The method is needed for teh scanline algorithm: in the case
        that a filling line intersects a boundary at a vertex, the
        vertex counts as boundary if it does, and does not count otherwise.
    
        This method is slow, hopefully it is very rarely used.
        
        TODO: right now, this assumes that scanlines are always oriented parallel
        with the x axis ( horizontal ). this algorithm needs a direction vector
        in order to be generalized for the situation where the fill lines can be
        oriented in any direction
    """
    #print "Determining if the specified point is a local minimum or not..."
    topexp = TopExp.TopExp_Explorer();
    topexp.Init(wire,TopAbs.TopAbs_EDGE);
    te = TopExp.TopExp();
    
    #find two edges that contain the vertex
    edges = [];
    while topexp.More():
        e = cast(topexp.Current() );
        if te.LastVertex(e).__hash__() == vertex.__hash__():
            #print "found a last vertex"
            edges.append(e);
        if te.FirstVertex(e).__hash__() == vertex.__hash__():
            edges.append(e);
            #print "found a first vertex"
        topexp.Next();
        
    assert  len(edges)  ==  2;
    
    #move a little away from the vertex in each direction
    
    ALITTLEBIT = d1 = d2 = 0.01;
    e1 = edges[0];
    e2 = edges[1];
    p1 = brepTool.Parameter( vertex, e1);
    p2 = brepTool.Parameter(vertex, e2) ;
    
    (bp1,ep1 ) = brepTool.Range(e1 );
    (bp2, ep2 ) = brepTool.Range(e2 );
    
    if p1 == ep1: d1 = d1*(-1);
    if p2 == ep2: d2 = d2*(-1);
    
    pnt1 = pointAtParameter(e1, p1  + d1);
    pnt2 = pointAtParameter(e2, p2 + d2 );
    
    #TestDisplay.display.showShape(vertex);    
    #TestDisplay.display.showShape(wire);    
    #TestDisplay.display.showShape(make_vertex(pnt1));
    #TestDisplay.display.showShape(make_vertex(pnt2));
    #TestDisplay.display.run();
    #finally! compare y values. this is the part that needs to change later
    #to account for rotated filling lines
    y = brepTool.Pnt(vertex).Y();
    
    dy1 = pnt1.Y() - y;
    dy2 = pnt2.Y() - y;
    
    if  dy1 * dy2 > 0:
        #print "local min/max detected."
        return True;
    else:
        #print "vertex is not a local min/max"
        return False;


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
            #    debugShape(make_vertex(p));
            #    allPoints.append(p);
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
    

def findPointOnCurve(point,handleCurve,tolerance):
    "return the closest parameter on the curve for the provided point"
    "returns none if a point is not within tolerance"
    gp = GeomAPI.GeomAPI_ProjectPointOnCurve(point,handleCurve);
    gp.Perform(point);
    if gp.NbPoints()>0 and gp.LowerDistance() <= tolerance:
        #log.debug( "Projection Success!" );
        return [gp.LowerDistanceParameter(),gp.NearestPoint()];
    else:
        #log.warn( "Projection Failed.");
        return None;

def makeWiresFromOffsetShape(shape):
    "get all the wires from the offset shape"
    resultWires = TopTools.TopTools_HSequenceOfShape();
    if shape.ShapeType() == TopAbs.TopAbs_WIRE:
        #log.info( "offset result is a wire" );
        wire = topoDS.wire(shape);
        #resultWires.append(wire);
        resultWires.Append(wire);
    elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
        #log.info( "offset result is a compound");

        bb = TopExp.TopExp_Explorer();
        bb.Init(shape,TopAbs.TopAbs_WIRE);
        while bb.More():
            #print (dir(topoDS))
            w = topoDS.wire(bb.Current());
                        
            #resultWires.append(w);
            resultWires.Append(w);#
            #debugShape(w);
            bb.Next();        
        bb.ReInit();        
    return resultWires;    

def wireFromEdges(edgeList):
    "TODO: might need to use sortwires to make sure it is i nthe right order"
    mw = BRepBuilderAPI.BRepBuilderAPI_MakeWire();
    for e in edgeList:
        mw.Add(e);
        
    if mw.IsDone():
        return mw.Wire();
    else:
        raise ValueError,"Error %d Building Wire" % mw.Error();

def pointAtParameterList(edge,paramList):
    "compute the point on the edge at the supplied parameter"
    hc = brepTool.Curve(edge);
    c = GeomAdaptor.GeomAdaptor_Curve(hc[0]);
    r = [];
    for p in paramList:
        r.append(c.Value(p));
    return r;

def endPoints(edge):
    "returns end points of an edge"        
    hc = brepTool.Curve(edge);
    curve = GeomAdaptor.GeomAdaptor_Curve(hc[0]); 
    return ( curve.Value(hc[1]), curve.Value(hc[2]));
    
def pointAtParameter(edge,param):
    hc = brepTool.Curve(edge);
    c = GeomAdaptor.GeomAdaptor_Curve(hc[0]);
    return c.Value(param);

def shortenEdge(edge, point, dist):
    "given an edge, shorten it by the specified distance on the end closest to point"
    (handleCurve, p1, p2  ) = brepTool.Curve(edge);
    curve = GeomAdaptor.GeomAdaptor_Curve(handleCurve);
    
    pt1 = curve.Value(p1)
    pt2 = curve.Value(p2)

    if pt1.SquareDistance(point) < pt2.SquareDistance(point):
        #pt1 is closer to point
        pstart = p1;
        pend = p2;
    else:
        #pt2 is closer to point
        pstart = p2;
        pend = p1;
    
    if pstart > pend:
        dist = (-1.0)*dist
  
    #compute a distance along the curve also
    gcpnts = GCPnts.GCPnts_AbscissaPoint(curve,pstart,dist);
    p2 = gcpnts.Parameter();
    return edgeFromTwoPointsOnCurve(handleCurve,p2,pend);
    
    
def splitEdge(edge, p):
    "splits an edge at the supplied parameter, returns the new edges"
    "separationDist is the distance of the break between the two edges"
    
    (handleCurve, pstart, pend  ) = BRep.BRep_Tool().Curve(edge);
    return ( edgeFromTwoPointsOnCurve(handleCurve,pstart,p), edgeFromTwoPointsOnCurve(handleCurve,p,pend));



def trimmedEdge(edge,p1, p2):
    "returns a new edge that is a trimmed version of the underlying one"
    hc = BRep.BRep_Tool().Curve(edge);
    return edgeFromTwoPointsOnCurve(hc[0],p1,p2);
    
def edgeFromTwoPointsOnCurve(handleCurve,p1,p2):
    "make an edge from a curve and two parameters"
    #log.info("Making Edge from Parameters %0.2f, %0.2f" %( p1, p2) );
    
    a = p1;
    b = p2;
    rev = False;
    if p1 > p2:
        a = p2;
        b = p1;
        #log.info("Parameters are inverted, so we'll reverse the resulting edge");
        rev = True;        
    
    builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(handleCurve,a,b);
    builder.Build();
    if builder.IsDone():
        e = builder.Edge();
        if rev:
            e.Reverse();
        return e;
    else:
        #log.error( "Error building Edge: Error Number %d" % builder.Error());
        raise ValueError,"Error building Edge: Error Number %d" % builder.Error();
        
def cast(shape):
    "these names have changed between occ 0.4 and occ 0.5!!!"

    type = shape.ShapeType();
    "cast a shape as its correct type"
    if type == TopAbs.TopAbs_WIRE:
        return topoDS.wire(shape);
    elif type == TopAbs.TopAbs_EDGE:
        return topoDS.edge(shape);
        #return TopoDS.TopoDS_edge(shape);
    elif type == TopAbs.TopAbs_VERTEX:
        return topoDS.vertex(shape);
    elif type == TopAbs.TopAbs_SOLID:
        return topoDS.solid(shape);
    elif type == TopAbs.TopAbs_FACE:
        return topoDS.face(shape);
    elif type == TopAbs.TopAbs_SHELL:
        return topoDS.shell(shape);
    elif type == TopAbs.TopAbs_COMPOUND:
        return topoDS.compound(shape);
    return shape;

#offset a list of wires
#returns a list of wires.
def offsetWireList(wireList,offsetAmount):

    bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
    for w in wireList:
        bo.AddWire(w);
    
    bo.Perform(offsetAmount,TOLERANCE);  #this line crashes hard, but only sometimes.
    #print "done offsetting..";
    if  not bo.IsDone():
        print "Warning: offset not computed!"
        return None;
    else:
        #make sure to return a list of wires also, since we got one in        
        returnWires = [];
        shape = bo.Shape();
        if shape.ShapeType() == TopAbs.TopAbs_WIRE:
            returnWires.append( cast(shape) );
        elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
        
            bb = TopExp.TopExp_Explorer();
            bb.Init(shape,TopAbs.TopAbs_WIRE);
            while bb.More():
                w = topoDS.wire(bb.Current());
                returnWires.append(w);
                bb.Next();        
            bb.ReInit();        
    return returnWires;            

#gets a list of wires from a face
def wireListFromFace(face):
    rW = []
    ow = brt.OuterWire(face);
    rW.append(ow);
    
    for w in Topo(face).wires():
        if not w.IsSame(ow):
            rW.append(w);
    
    return rW;

#offset a single wire    
def offsetWire(wire,offsetAmount ):
    TOLERANCE = 0.0005;
    
    #print wire;
    bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset(wire,GeomAbs.GeomAbs_Line);
    #bo.AddWire(wire);
    
    #print "about to offset by %0.2f" % offset;
    bo.Perform(offsetAmount,TOLERANCE);  #this line crashes hard, but only sometimes.
    #print "done offsetting..";
    if  not bo.IsDone():
        raise Exception, "Offset Was Not Successful.";
    else:
        return  bo.Shape();

#offset a face, returning the offset shape
"""
    We may want to speed things up later using 2-d offsets,
    but this is ok for a start
"""
def offsetFace(face,offsetAmount ):
    ow = brt.OuterWire(face);
    bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();
    bo.AddWire(ow);
    
    for w in Topo(face).wires():
        if not w.IsSame(ow):
            bo.AddWire(w);
    
    print "about to offset by %0.2f" % offsetAmount;
    bo.Perform(offsetAmount,TOLERANCE);  #this line crashes hard, but only sometimes.
    #print "done offsetting..";
    if  not bo.IsDone():
        print "Warning: offset not computed!"
        return None;
    else:
        return  bo.Shape();

def isFaceAtZLevel(zLevel,face):
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


if __name__=='__main__':
    
    from OCC.Display.SimpleGui import *
    display, start_display, add_menu, add_function_to_menu = init_display()
    
    #test trimming and such
    e1 = edgeFromTwoPoints ( gp.gp_Pnt(0,0,0), gp.gp_Pnt(1,1,0));
    display.DisplayColoredShape(e1,'BLUE');
    
    #just for testing, get the parameters of the new edge.
    (handleCurve, p1, p2  ) = BRep.BRep_Tool().Curve(e1);
    p = (p1 + p2) / 2 
    display.DisplayColoredShape( splitEdge(e1,p,0.1), 'RED');
    #display.DisplayColoredShape( shortenEdge(e1,p1,0.1), 'WHITE');
    display.DisplayColoredShape( shortenEdge(e1,p2,0.1), 'WHITE');
    
    start_display();
    