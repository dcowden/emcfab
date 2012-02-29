"""
    Constructs test objects for testing support
"""

from OCC import gp,BRepBuilderAPI,GC

import OCCUtil

def makeSquareWire():
    "this is a square"
    p1 = gp.gp_Pnt(0,0,0);
    p2 = gp.gp_Pnt(5,0,0);
    p3 = gp.gp_Pnt(5,5,0);
    p4 = gp.gp_Pnt(0,5,0);
    e1 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2).Edge();
    e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p2,p3).Edge();
    e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p3,p4).Edge();
    e4 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p4,p1).Edge();
    mw = BRepBuilderAPI.BRepBuilderAPI_MakeWire();
    mw.Add(e1);
    mw.Add(e2);
    mw.Add(e3);
    mw.Add(e4);
    return mw.Wire();


def makeKeyHoleWire():
    circle2 = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(40,40,2),gp.gp_Dir(0,0,1)),10)
    Edge4 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle2,gp.gp_Pnt(40,50,2),gp.gp_Pnt(50,40,2)).Edge()
    ExistingWire2 = BRepBuilderAPI.BRepBuilderAPI_MakeWire(Edge4).Wire()
    P1 = gp.gp_Pnt(50,40,2)
    P2 = gp.gp_Pnt(80,40,2) #5,204,0
    Edge5 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(P1,P2).Edge()
    MW = BRepBuilderAPI.BRepBuilderAPI_MakeWire()
    MW.Add(Edge5)
    MW.Add(ExistingWire2)

    if MW.IsDone():
        WhiteWire = MW.Wire()
        return [WhiteWire,Edge5,ExistingWire2];    
    
def makeReversedWire():
    "this is a square"
    p1 = gp.gp_Pnt(.5,.5,0);
    p2 = gp.gp_Pnt(1,4,0);
    p3 = gp.gp_Pnt(2,4,0);
    e1 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2).Edge();
    e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p3,p2).Edge();
    e2.Reverse();
    e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p3,p1).Edge();
    mw = BRepBuilderAPI.BRepBuilderAPI_MakeWire();
    mw.Add(e1);
    mw.Add(e2);
    mw.Add(e3);
    return mw.Wire();    
    
def makeCircleWire():
    "designed to be include inside the square to simulate an island"
    
    circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(2,2,0),gp.gp().DZ()),1);
    e1 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle).Edge();
    mw = BRepBuilderAPI.BRepBuilderAPI_MakeWire();
    mw.Add(e1);
    return mw.Wire();   

def makeHeartWire():
    "make a heart wire"
    e1 = OCCUtil.edgeFromTwoPoints(gp.gp_Pnt(0,0,0), gp.gp_Pnt(4.0,4.0,0));
    circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(2,4,0),gp.gp().DZ()),2);
    e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle, gp.gp_Pnt(4,4,0),gp.gp_Pnt(0,4,0)).Edge();
    circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(-2,4,0),gp.gp().DZ()),2);
    e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle, gp.gp_Pnt(0,4,0),gp.gp_Pnt(-4,4,0)).Edge();
    e4 = OCCUtil.edgeFromTwoPoints(gp.gp_Pnt(-4,4,0), gp.gp_Pnt(0,0,0));
    return OCCUtil.wireFromEdges([e1,e2,e3,e4]);

def makeOffsetTestWire():
    "creates difficult test cases for offsetting"
    p1 = OCCUtil.pnt(11.0,0);
    p2 = OCCUtil.pnt(7.0,8.0);
    p3 = OCCUtil.pnt(7.0,12.0);
    p4 = OCCUtil.pnt(17.0,22.0);
    p5 = OCCUtil.pnt(0.0,22.0);
    p6 = OCCUtil.pnt(3.0,17.0);
    p7 = OCCUtil.pnt(4.0,8.0 );
    c1 = OCCUtil.pnt(10.0,18.5);
    c2 = OCCUtil.pnt(6.0,3.0);
    
    edges = [];
    edges.append( OCCUtil.edgeFromTwoPoints( p1, p2 ));
    edges.append( OCCUtil.edgeFromTwoPoints( p2, p3 ));
    
    circle = GC.GC_MakeArcOfCircle(p3, c1, p4); #circle through 3 points
    e2 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle.Value()).Edge();
    edges.append(e2);
    
    edges.append( OCCUtil.edgeFromTwoPoints( p4, p5 ));
    edges.append( OCCUtil.edgeFromTwoPoints( p5, p6 ));
    edges.append( OCCUtil.edgeFromTwoPoints( p6, p7 ));

    circle = GC.GC_MakeArcOfCircle(p1, c2, p7 ); #circle through 3 points
    e3 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle.Value() ).Edge();
    edges.append(e3);

    return OCCUtil.wireFromEdges(edges);

def makeFaceWithHold():
    "creates a face with a thin feature due to a hole"
    ow = makeSquareWire(); #box 0,0 --> 5,5
    
    p1 = OCCUtil.pnt(0.4,0.4);
    p2 = OCCUtil.pnt(2.0,0.4);
    p3 = OCCUtil.pnt(2.0,2.0);
    p4 = OCCUtil.pnt(2.0,0.4);
    
    


if __name__=='__main__':
    
    from OCC.Display.SimpleGui import *
    display, start_display, add_menu, add_function_to_menu = init_display()
    
    #display.DisplayShape(makeCircleWire());
    #isplay.DisplayShape(makeHeartWire());
    #display.DisplayShape(makeReversedWire());
    #display.DisplayShape(makeKeyHoleWire());
    #display.DisplayShape(makeSquareWire());
    display.DisplayShape(makeOffsetTestWire());
    display.FitAll();
    start_display()

    