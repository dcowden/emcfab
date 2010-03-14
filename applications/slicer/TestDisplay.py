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
class AppFrame(wx.Frame):
  def __init__(self, parent,title,x,y):
      wx.Frame.__init__(self, parent=parent, id=-1, title=title, pos=(x,y),style=wx.DEFAULT_FRAME_STYLE,size = (400,300))
      self.canva = wxViewer3d(self);
  def showShape(self,shape):
		self.canva._display.DisplayShape(shape)
  def eraseAll(self):
  		self.canva._display.EraseAll();


def makeTestWire():
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
	return WhiteWire;	
	
if __name__=='__main__':	
	app = wx.PySimpleApp()
	wx.InitAllImageHandlers()
	debugDisplay = AppFrame(None,"Debug Display",1220,20)
	debugDisplay.canva.InitDriver()
	debugDisplay.canva._display.SetModeWireFrame()
	debugDisplay.Show(True)	
	
	debugDisplay.showShape(makeTestWire());
	
	app.SetTopWindow(debugDisplay)
	app.MainLoop() 