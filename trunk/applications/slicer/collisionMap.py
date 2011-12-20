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
from OCC import BRep;
from OCC import TopTools
from OCC import TopoDS
from OCC import TopAbs
from OCC import TopExp
from OCC import BRepTools
from OCC import TColgp
from OCC import Poly
from OCC import TopLoc
from OCC import BRepMesh
from OCC.Utils.Topology import Topo
from OCC import BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor,IntCurvesFace,Approx,BRepLib
import pixmaptest
import Wrappers
import TestWires
import TestDisplay
import pixMapTileTest
import breshamtest as bres
#import bresenham as bres
import numpy as np
import networkx as nx
import hexagonlib

ts = TopoDS.TopoDS();
btool = BRep.BRep_Tool();
brt = BRepTools.BRepTools();