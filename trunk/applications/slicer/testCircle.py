import os
import sys
import os.path
import wx
import logging
import time
import traceback

import math
from OCC  import gp
from OCC  import BRepAdaptor
from OCC import BRepBuilderAPI
from OCC import Extrema
from OCC import Approx
from OCC import GeomAbs
from OCC import BRepExtrema
from OCC import Geom
from OCC import BRepBndLib 
from OCC import Bnd;
from OCC import TopTools
from OCC import TopoDS
from OCC import TColStd

import Wrappers
import TestDisplay

if __name__=='__main__':

	circle = gp.gp_Circ(gp.gp_Ax2(gp.gp_Pnt(0,0,0),gp.gp().DZ()),1);
	
	#make a few edges
	e1 = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle,5,6.2).Edge();
	TestDisplay.display.showShape(e1);
	
	TestDisplay.display.run();