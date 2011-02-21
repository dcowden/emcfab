import os
import sys
import os.path
import wx
import logging
import time
import traceback
import math

#OCC libraries
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
from OCC import BRepTools
from OCC.Display.SimpleGui import *

#emcfab modules
import Wrappers
import hexagonlib

#occ display for debugging
display, start_display, add_menu, add_function_to_menu = init_display()

"""
	An undirected graph of edges.

"""

def hashE(edge):
	return edge.HashCode(1000000);

class EdgeNode:
	"an edge that will be traced to fill"
	
	def __init__(self,edge,type):
		"edge is the OCC native edge object"
		"type is an edge type, either FILL or BOUND"
		
		self.type = "BOUND";
		self.edge = edge;
		self.id = hashE(self.edge);
		self.edgeWrapper = Wrappers.Edge(edge);
		
		self.nextFill = None;
		self.prevFill = None;
		
		self.nextBoundary = None;
		self.prevBoundary = None;
	
	
	def intersectBoundary(self, otherEdge, thisParam, otherParam):
		"""
		given two edges that intersect, split them into four edges
		otherEdge is a FillEdge object.
		thisParam is the parameter on this edge
		otherParam is the parameter on the other edge
		
		returns the four newly created edges, though probably you dont care.
		
		The current edge and underlying objects are not useful after this operation.
		For example, any wires containing these edges are probably messed up now.
		
		"""
		
		if self.type == 'BOUND':
			raise ValueError,"Shold be called on fill edges only"
			
		if otherEdge.type == 'FILL':
			raise ValueError,"Supplied edge should be a boundary"
	
		#split this edge
		es = self.edgeWrapper.splitAtParameter(thisParam); #two edgewrappers
		bs = self.edgeWrapper.splitAtParameter(otherParam); #two edgewrappers

		#create new edges.
		f1 = EdgeNode(es[0],'FILL');
		f2 = EdgeNode(es[1],'FILL');
		
		b1 = EdgeNode(bs[0],'BOUND');
		b2 = EdgeNode(bs[1],'BOUND');
		
		#rewire fill edges
		f1.prevFill = self.prevFill;
		f1.nextFill = f2;
		f2.prevFill = f1;
		f2.nextFill = self.nextFill;

		#re-wire boundary
		b1.prevBoundary = otherEdge.prevBoundary;
		b1.nextBoundary = b2;
		b2.prevBoundary = b1;
		b2.nextBoundary = otherEdge.nextBoundary;
		
		#connect the nodes together. done with the right-hand rule: next is right, left is prev
		f1.nextBoundary = b2;
		f1.prevBoundary = b1;
		f2.nextBoundary = b1;
		f2.prevBoundary = b2;
		
		b1.nextFill =  f1;
		b1.prevFill = f2;
		b2.nextFill = f2 ;
		b2.prevFill = f1 ;
		
		return [f1,f2,b1,b2];
		