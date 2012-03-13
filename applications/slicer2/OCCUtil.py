"""
    Utility functions related to OCC
"""
import time,os,sys,string,math;

from OCC import BRep,gp,GeomAbs,Geom,GeomAPI,GCPnts,TopoDS,BRepTools,GeomAdaptor,TopAbs,TopTools,TopExp,Approx,BRepLib
from OCC import BRepGProp,BRepLProp, BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
from OCC import BRepExtrema,TColgp
from OCC import ShapeAnalysis
from OCC import ShapeFix,ShapeExtend
from OCC.Utils.Topology import Topo

from Constants import *;

brt = BRepTools.BRepTools();
brepTool = BRep.BRep_Tool();
topoDS = TopoDS.TopoDS();

import Wrappers
import itertools
import TestObjects
import Util

def pnt(x,y):
    return gp.gp_Pnt(x,y,0);

def make_vertex(pnt):
    "make a vertex from a single point"
    s = BRepBuilderAPI.BRepBuilderAPI_MakeVertex(pnt);
    s.Build();
    return s.Shape();

def listFromTopToolsListOfShape(listOfShape):
    newList = [];
    iterator = TopTools.TopTools_ListIteratorOfListOfShape(listOfShape);
    while iterator.More():
        newList.append(cast(iterator.Value()));
        iterator.Next();
        
    return newList;

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

def childShapes(shape):
    """
        List immediate child shapes of the provided shape
    """
    children = [];
    it = TopoDS.TopoDS_Iterator(shape)
    while it.More():
        children.append(cast(it.Value()));
        it.Next()
    return children;
    
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
      

def paramAtDistanceFromPoint(curve,param,distance):
    
    #gcpnts = GCPnts.GCPnts_AbscissaPoint(curve,param,distance);
    #return gcpnts.Parameter(); 
    return param + distance;

def divideEdgeAtParam(edge, param, distance,separation):
    """
        Divides an edge.
        Edge is the edge to divide
        param is the parameter on the edge that is the basis for the division
        dist is the distance from the supplied parameter to move along the curve from the supplied parameter
        separateWidth is the gap along the curve that should be left between the divided edges
        
        distance will be at least twice the separation distance
        returns a list of edges.  there will be two edges if the sparation point occurs in the middle of
        the edge, but only one if the separation point was too close to a vertex ( in which case, only one is returned )
        if only one edge is returned, the vertex closer to the desired point should be used
        
        NOTE: curves are normally paramterized by distance, so it is a good assumption, though not always true, that
        shortest distance is shorted difference between parameters. there is a good chance that simply subtracting parameters would
        actually work!!!!
    
        
        
    """
    (handleCurve, p1, p2  ) = brepTool.Curve(edge);
    curve = GeomAdaptor.GeomAdaptor_Curve(handleCurve);
        
    if p1>p2: (p1,p2) = (p2,p1) #now p1 < p2 
    
    assert param > p1 and param < p2; #we expect param between p1 and p2.
    
    if distance < (2.0)*separation:
        distance = (2.0)*separation;
    
    newP = paramAtDistanceFromPoint(curve,param,distance);
    
    if newP < p1 or newP > p2:
        #requested distance moved us off the curve. try moving the other way on the curve
        newP = paramAtDistanceFromPoint(curve,param,(-1.0)*distance);

        if newP < p1: #still off the curve
            newP = p1;
            newP2 = paramAtDistanceFromPoint(curve,p1,separation);
            return [edgeFromTwoPointsOnCurve(handleCurve,newP2,p2)];
        if newP > p2:
            newP = p2;
            newP2 = paramAtDistanceFromPoint(curve,newP,(-1.0)*separation);
            return [edgeFromTwoPointsOnCurve(handleCurve,p1,newP2)];
    else:
        #requested location was found between the parameters. find other point
        newP2 = paramAtDistanceFromPoint(curve,newP,separation);
        return [edgeFromTwoPointsOnCurve(handleCurve,p1,newP),  edgeFromTwoPointsOnCurve(handleCurve,newP2,p2) ];
        
    return (  edgeFromTwoPointsOnCurve(handleCurve),  edgeFromTwoPointsOnCurve(handleCurve) );
        

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

    if len(wireList) == 0:
        print "Warning: zero wires in the shape-- skipping"
        return [];
    
    bo = BRepOffsetAPI.BRepOffsetAPI_MakeOffset();

    for w in wireList:
        bo.AddWire(w);

    bo.Perform(offsetAmount,0.0);  #if this crashes, try using a small non zero number for the last argument
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
                returnWires.append(cast(w));
                bb.Next();        
            bb.ReInit();        
    return returnWires;            

"""
    returns the closest vertex to the provided point, on the provided wire list 
    as well as the edge it is on.
    This method appeared faster than BRepExtremaDistShapeShape
    would projecting the point be faster?
"""
def nearestVertex(wireList,point):
    dist = 9999999;
    (nw,nv) = (None,None);
    for w in wireList:
        tw = Topo(w);
        for v in tw.vertices():                    
            p = brepTool.Pnt(v);
            if point.Distance(p) < dist:
                dist = point.Distance(p);                        
                (nw,nv) =  (w,v );          
    return ( nw,nv);

   
"""
    Given a wire and a vertex, return a new wire
    that is trimmed so that there is no overlap
    
    returns (new wire, edge that was trimmed, point on the edge )
    new wire will not be closed
    
    TODO: this version of the method simply moves along the cuves by the trimWidth.
    though this is easy, it is not most accurate, especially for curves that have high
    overlap ( like very thin things ). There is a lot of code computing normal vectors here,
    this code is still here, though not necessary if we take this simple approach.
    
    The shortenEdge function is simpler if we dont care about accounting for the angles involved.
"""
def getTrimmedWire(wire,startPoint, trimWidth):

    (w,vertex) = nearestVertex([wire],startPoint);
    
    eL = []
    tw = Topo(wire);
    for e in tw.edges_from_vertex(vertex):
        eL.append(e);
      
    if len(eL) > 2:  raise Exception("Cannot trim wire at vertex with more than two edges");    
    if len(eL) == 0:  raise Exception("Vertex has no edges, cannot trim")
    
    if not brepTool.IsClosed(wire):
        raise Exception ("Do not know how to trim a wire that is not closed!")

    #if there is only one edge, it must be a circle or spline, because it is still closed
    #we just need to trim the end a bit
    if len(eL) == 1:
       edge1 = eL[0]; 
       (hCurve1, startp1, endp1) = brepTool.Curve(edge1);
       curve1 = hCurve1.GetObject();
       vPoint = brepTool.Pnt(vertex);
       pEnd = endp1 - (trimWidth / 2.0 );
       newEdge = trimmedEdge(edge1,startp1, pEnd );
       
       return (w,newEdge,curve1.Value(pEnd));
   
    #else len(eL) == 2
    edge1 = eL[0];
    edge2 = eL[1];
    def D1AtPoint(curve,p):
        qq = gp.gp_Pnt();
        tt = gp.gp_Vec();
        curve.D1(p,qq,tt);
        return ( qq,tt);
    
    #measure the angle between the wires
    #TODO: probably can factor this out somewhere. didnt, becaues
    #moving the logic might result in getting Curve objects more
    (hCurve1, startp1, endp1) = brepTool.Curve(edge1);
    (hCurve2, startp2, endp2) = brepTool.Curve(edge2);

    curve1 = hCurve1.GetObject();
    curve2 = hCurve2.GetObject();
    #print dir(curve1)
    #print dir(hCurve1)
    vPoint = brepTool.Pnt(vertex);

    #TODO: performance improvement/refactor? right now this code is organized to be 
    #shortest but in the line case we do two computations    

    #which end of each edge is at the vertex? I think we should be able to know by
    #some convention


    curve2Reversed = False;
    curve1Reversed = False;
    (p1,v1) = D1AtPoint(curve1,startp1);
    if p1.Distance(vPoint) > 0.001:
        #recompute, we picked wrong end
        curve1Reversed = True;
        (p1,v1) = D1AtPoint(curve1,endp1);
        
    (p2,v2) = D1AtPoint(curve2,startp2);
    if p2.Distance(vPoint) > 0.001:
        #recompute, we picked wrong end
        curve2Reversed = True;
        (p2,v2) = D1AtPoint(curve2,endp2);
        
    if curve1.IsKind( "Geom_Line" ) and curve2.IsKind( "Geom_Line" ):
        #ok now in theory v1 and v2 represent the slopes
        #print "Both Curves are Lines"     
        angle = abs( math.pi - v1.Angle(v2));
        
        if angle == 0:
            print "Trimmed Wire: two edges are lines and are right on top of each other!"
            #remove curve two completely.
            trimDistance = endp2 - startp2;
        elif  angle < (math.pi / 2):
            trimDistance =  trimWidth / 2 * (1 + (1 / math.sin(angle)));
        else:
            trimDistance = trimWidth /2 ;
    else:
        #print "At least one curve is a curve"
        #at least one of the curves is not a line. here, we'll sample each curve looking for where they
        # are apart by the correct distance
        #find parameterization that will step along both curves starting with the vertex
        WIDTH = trimWidth / 8.0;
        if curve1Reversed: (q1,q2,qstep) = (startp1,endp1,WIDTH);
        else: (q1,q2,qstep) = (endp1,startp1,(-1.0)*WIDTH);
        
        if curve2Reversed: (s1,s2,sstep) = (startp2,endp2,WIDTH);
        else: (s1,s2,sstep) = (endp2,startp2,(-1.0)*WIDTH);
        
        print "curve1: %d %d %0.3f" % ( q1, q2, qstep)
        print "curve2: %d %d %0.3f" % ( s1, s2, sstep)
        
        distance = 0.0;
        q = q1;
        s = s1;
        while distance < trimWidth and q != q2 and s != s2:
            
            distance = curve1.Value(q).Distance(curve2.Value(s));
            q += qstep
            s += sstep;
        print "distance= %0.3f" % distance
        #when we exit the loop, q and s are about WIDTH apart on curves 1 and 2
        trimDistance = s - s1;
        print "trimDistance = %0.3f" % trimDistance
            
    #for now, arbitrarily shorten edge 2, return wire following it around
    #starting with edge 1
    if curve2Reversed:
        pTrimStart = startp2
        pTrimEnd = endp2 - trimDistance
    else:
        pTrimStart = startp2 + trimDistance
        pTrimEnd = endp2;

    newE = trimmedEdge(edge2,pTrimStart,pTrimEnd);
    
    #find point at the end. there must be a better way to do this, but
    #somehow the edge is getting reversed somewhere!
    #here we just find the vertex closest to the original vertex point
    dist=99999;
    lastP = None;
    for v in Topo(newE).vertices():
        p = brepTool.Pnt(v);
        if p.Distance(vPoint) < dist:
            lastP = p;
            dist = p.Distance(vPoint);
            
    #now build up the edges in order so we can return the wire
    wB = WireBuilder();
    wB.add(edge1);
    
    #add the edges we didnt touch
    for e in tw.edges():
        if not e.IsSame(edge1) and not e.IsSame(edge2 ):
            wB.add(e);

    wB.add( newE); #the last trimmed edge
    return ( wB.wire(), edge2, lastP );
    
    
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

"""

    Smart Wire Builder..
    accepts edges, and then uses WireOrder to connect them together into the best possible order.
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
        #print "Ordering Wires. We have %d Edges" % len(self.edges)
        self.fixer.Load(self.wireData.GetHandle());
        self.fixer.SetClosedWireMode(True)
        self.fixer.FixReorder();
        self.fixer.FixConnected();
        self.fixer.FixClosed();
        return self.fixer.Wire();
    
def TestShortenEdge():
    #test trimming and such
    e1 = edgeFromTwoPoints ( gp.gp_Pnt(0,0,0), gp.gp_Pnt(1,1,0));
    display.DisplayColoredShape(e1,'BLUE');
    
    #just for testing, get the parameters of the new edge.
    (handleCurve, p1, p2  ) = BRep.BRep_Tool().Curve(e1);
    p = (p1 + p2) / 2 
    display.DisplayColoredShape( splitEdge(e1,p,0.1), 'RED');
    #display.DisplayColoredShape( shortenEdge(e1,p1,0.1), 'WHITE');
    display.DisplayColoredShape( shortenEdge(e1,p2,0.1), 'WHITE');    

def TestTrimmedWire():
    # test triming a wire
    #this will select each vertex of the heart.
    points = [];
    points.append( gp.gp_Pnt(0,0,0));
    points.append( gp.gp_Pnt(4,4,0));
    points.append( gp.gp_Pnt(-4.2,4,0));
    points.append( gp.gp_Pnt(0,3.5,0));


    h = TestObjects.makeHeartWire();
    
    for p in points:
        display.EraseAll();
        display.DisplayColoredShape(h, 'GREEN');        
        
        (w2,edge,p2) = getTrimmedWire(h,p,0.5);
        display.DisplayColoredShape(w2,'RED');
        display.DisplayColoredShape(make_vertex(p2),'RED');
        time.sleep(5);

        
def TestDivideEdge():
    #lines are parameterized between 0 and line length
    edge = edgeFromTwoPoints ( gp.gp_Pnt(0,0,0), gp.gp_Pnt(2,2,0));
    (handleCurve, p1, p2  ) = brepTool.Curve(edge); #p1=0, #p2=2.828
    print p1,p2
    totalLen = (p2 - p1);
    display.DisplayColoredShape(edge, 'BLUE');
    #divide the edge into two edges, split at 0.25 units from param=1.414, with a gap of 0.1 between them
    edges = divideEdgeAtParam(edge, 1.414, 0.25,0.1); #divide the edge into two edges
    assert(len(edges)== 2);
    (e1,e2) = ( edges[0],edges[1]);
    (hc,q1,q2) = brepTool.Curve(e1);
    print q1,q2;

    (hc,q1,q2) = brepTool.Curve(e2);
    print q1,q2;
    print Wrappers.Edge(e1).distanceBetweenEnds();
    assert (abs(Wrappers.Edge(e1).distanceBetweenEnds()- (1.414 + 0.25)) < 0.000001);
    assert (abs(Wrappers.Edge(e2).distanceBetweenEnds() - (totalLen - 1.414 - 0.25 - 0.1)) < 0.0000001 ); 
    display.DisplayColoredShape(edges,'RED');
    
    edges = divideEdgeAtParam(edge, 1.414, 2.5,0.1); #should return one edge of length 0.1 less than total
    assert ( len(edges) ==1);
    assert ( abs(Wrappers.Edge(edges[0]).distanceBetweenEnds() - (totalLen - 0.1)) < 0.0000001 );
    display.DisplayColoredShape(edges,'GREEN');
if __name__=='__main__':
    
    from OCC.Display.SimpleGui import *
    display, start_display, add_menu, add_function_to_menu = init_display()
    
    TestTrimmedWire();
    
    start_display();
    