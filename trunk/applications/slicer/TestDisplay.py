"""
  for displaying a wire or other test shapes
"""
import os
import sys
import os.path
import wx
import logging
import time
import traceback
from OCC import STEPControl,TopoDS, TopExp, TopAbs,BRep,gp,GeomAbs,GeomAPI
from OCC import BRepBuilderAPI,BRepTools,BRepAlgo,BRepBndLib,Bnd,StlAPI,BRepAlgoAPI,BRepAdaptor
from OCC import BRepLProp,BRepGProp,BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepExtrema
from OCC import BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor,IntCurvesFace,Approx,BRepLib
from OCC.Display.wxDisplay import wxViewer3d

import Wrappers

class AppFrame(wx.Frame):
  def __init__(self, parent,title,x,y,app):
      wx.Frame.__init__(self, parent=parent, id=-1, title=title, pos=(x,y),style=wx.DEFAULT_FRAME_STYLE,size = (400,300))
      self.canva = wxViewer3d(self);
      self.app = app;
  def showShape(self,shape):
		self.canva._display.DisplayShape(shape)
  def showShapes(self,listOfShape):
		for s in listOfShape:
			self.canva._display.DisplayShape(s);
			
  def eraseAll(self):
  		self.canva._display.EraseAll();
  def run(self):
	self.app.MainLoop();

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

def makeEdgeIndicator(edge):
	"makes an indicator showing which way the edge goes"
	ew = Wrappers.Edge(edge);
	fp = ew.firstParameter;
	lp = ew.lastParameter;
	
	if ew.reversed:
		p = fp + (( lp - fp ) * 1 / 5 );
	else:
		p = fp + ((lp - fp) * 4 /5 );
	midPnt = ew.curve.Value(p);
	return Wrappers.make_vertex(midPnt);

	
app = wx.PySimpleApp()
wx.InitAllImageHandlers()
display = AppFrame(None,"Test Debug Display",1220,20,app)
display.canva.InitDriver()
display.canva._display.SetModeWireFrame()
display.Show(True)	
app.SetTopWindow(display);