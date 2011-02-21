"""
   Copyright [2009] [Dave Cowden ( dave.cowden@gmail.com)]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

	Contains routines from OccSliceLib that are no longer needed, but
	this way i wont have to delete them.
	
	OCC Libraries seem to be hard to code, so this way i can refer to these
	if i have to re-implement any of these.
	

"""



from OCC import STEPControl,TopoDS, TopExp, TopAbs,BRep,gp,BRepBuilderAPI,BRepTools,BRepAlgo,BRepBndLib,Bnd,StlAPI,BRepAlgoAPI
from OCC import BRepLProp,BRepGProp,BRepBuilderAPI,BRepPrimAPI,GeomAdaptor,GeomAbs,BRepClass,GCPnts,BRepBuilderAPI,BRepOffsetAPI,BRepAdaptor
import os
import sys
import os.path
import wx
import logging
import time
import traceback
import Gcode_Lib
import Topology
from OCC.Display.wxDisplay import wxViewer3d
from OCC.Utils.Topology import Topo
from OCC.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.ShapeAnalysis import ShapeAnalysis_WireOrder
from OCC.ShapeFix import ShapeFix_Wire
from Cheetah.Template import Template
from OCC import ShapeFix
from OCC import BRepBuilderAPI
from OCC import TopTools


###Logging Configuration
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s',
                    stream=sys.stdout)

##
##  TODO:
##   X change pathmanager to allow other formats and easy structure access
##   X use builtin wxDisplay
##   X change naming of slice() to something else
##   X recognize .stp in addition to .step
##   X install guide
##   X remove unneed dumpTopology and shapeDescription
##     add 2d per-slice view on select of slice
##   X add ability to easily select slice thickness
##     add separate display for original object and slices
##   X sew faces from crappy stl files into single faces somehow
##   X remove reference to profile import



#####
#utility class instances
#available to all methods
#####
brt = BRepTools.BRepTools();
btool = BRep.BRep_Tool();
ts = TopoDS.TopoDS();
topexp = TopExp.TopExp()
texp = TopExp.TopExp_Explorer();
BRepLProp_CurveTool = BRepLProp.BRepLProp_CurveTool();

	
"""
	Sort a list of wires, by connecting them into paths if possible.
	@TODO: note, if this works, it also means that we might be able to use brepAlgo_Section 
	instead of solid operations.  We were just using solids because we get a compound instead of
	wires. but if we can figure out how to sort wires into paths, well-- this would work..
"""
def sortWires( compound ):
	sf = ShapeAnalysis_FreeBounds();
	#Topology.dumpTopology(compound);
	inputwires = TopTools.TopTools_HSequenceOfShape();
	print "Sorting Wires..."
	#add all the wires in the compound to the input wires
	texp = TopExp.TopExp_Explorer();
	texp.Init(compound,TopAbs.TopAbs_WIRE);

	while ( texp.More() ):
		wire = ts.Wire(texp.Current());
		inputwires.Append(wire);
		texp.Next();
	
	#free memory
	texp.ReInit();

	print "There are ",inputwires.Length()," wires."
	outputwires = TopTools.TopTools_HSequenceOfShape();
	print "New list has",outputwires.Length()," wires."
	#print inputwires.GetHandle()
	#print outputwires.GetHandle()
	sf.ConnectWiresToWires(inputwires.GetHandle(),0.0001,False,outputwires.GetHandle() );
	
	#for now return the input wires, which seems to work at least
	return inputwires;

"""
   Manages a set of loops and points
   a loop is chain of points that end where it begins
   a slice can be composed of multiple faces.
"""
class Loop:
	def __init__(self):
		self.points = [];
		self.pointFormat = "%0.4f %0.4f ";
		self.tolerance = 0.00001;
		
	def addPoint(self,x,y):
		p = gp.gp_Pnt2d(x,y);
		self.points.append( p );
	

	"""
		Print SVG Path String for a loop.
		If the end point and the beginning point are the same,
		the last point is removed and replaced with the SVG path closure, Z
	"""
	def svgPathString(self):
		lastPoint = self.points.pop();		
		if self.points[0].IsEqual( lastPoint,self.tolerance ):
			closed = True;
		else:
			closed = False;
			self.points.append(lastPoint);
		
		s = "M ";
		p = self.points[0];
		s += self.pointFormat % ( p.X(), p.Y() );
		for p in self.points[1:]:
			s += "L ";
			s += self.pointFormat % ( p.X(),p.Y())
		
		if closed:
			s+= " Z";
		
		return s;
	
			

######
# Decorates a slice to provide extra computations for SVG Presentation.
# Needed only for SVG display
######	
class SVGLayer:
	def __init__(self,slice,unitScale,numformat):
		self.slice = slice;
		self.margin = 20;
		self.unitScale = unitScale;
		self.NUMBERFORMAT = numformat;
	def zLevel(self):
		return self.NUMBERFORMAT % self.slice.zLevel;
		
	def xTransform(self):
		return self.margin;
		
	def yTransform(self):
		return (self.slice.layerNo + 1 ) * (self.margin + ( self.slice.sliceHeight * self.unitScale )) + ( self.slice.layerNo * 20 );


"""
	fixWire2
	fixes a wire by reordering it with shapeanalysis_wireorder
"""
def fixWire2(wire):
	swo = ShapeAnalysis_WireOrder();
	#swo.SetMode(False,0.000001);
	
	#for each edge, add beginning and end
	bwe = BRepTools.BRepTools_WireExplorer(wire);
	
	#store the edges in a sequence
	edges = TopTools.TopTools_HSequenceOfShape();
	
	while bwe.More():
		edge = bwe.Current();
		edges.Append(edge);
		curve = BRepAdaptor.BRepAdaptor_Curve(edge);		
		p1 = curve.FirstParameter();
		p2 = curve.LastParameter();	
		
		#get first and last points.
		firstPoint = gp.gp_Pnt();
		lastPoint = gp.gp_Pnt();
		BRepLProp_CurveTool.Value(curve,p1,firstPoint );
		BRepLProp_CurveTool.Value(curve,p2,lastPoint );			

		swo.Add(firstPoint.XYZ(),lastPoint.XYZ());
		
		bwe.Next();
	bwe.Clear();
	
	print "There are ",edges.Length()," edges in this wire"
	#now the edges are order
	swo.Perform();
	result = swo.Status();
	print "ordering complete. result is ",result
	
	#ok now the tricky part. Re-order the edges
	#the ordered method returns the original edge number
	edgeMap = {};
		
	for i in range(1,edges.Length()+1):
		newNum = swo.Ordered(i);
		print "New Edge",i," = original",newNum
		#oEdge = edges[abs(newNum)-1];
		#if newNum < 0:
		#	oEdge.Reverse();
		edgeMap[i] = newNum;
		
		
	print ("EdgeMap:",edgeMap);
	
	#now the new edges are in order
	newEdgeList = TopTools.TopTools_HSequenceOfShape();
	pyEdges = [];
	for i in range(1,edges.Length()+1):
		ei = edgeMap[i];	
		d = edges.Value(abs(ei));
		edge = ts.Edge(d);
		if ei < 0:
			edge.Reverse();
			print "Reversing Edge",i

		newEdgeList.Append(edge);
		pyEdges.append(edge);
		
	#print edgeList;
	#return newEdgeList;
	print "After Sort:"
	summarizeEdgeList(pyEdges);
	return pyEdges;
	
"""
	Writes a sliceset to specified file in Gcode format.
	TODO: this needs to extend a base class, it duplicates a lot of code in SVGExporter
"""		
class GcodeExporter ():
	def __init__(self,sliceSet):
		self.sliceSet = sliceSet
		self.title="Untitled";
		self.description="No Description"
		self.feedRate = 100;
		self.units = sliceSet.analyzer.guessUnitOfMeasure();
		self.NUMBERFORMAT = '%0.3f';
		
		self.frame = AppFrame(None,"Generated Toolpaths" + filename,420,20)
		self.frame.canva.InitDriver()
		self.frame.canva._display.SetModeWireFrame()
		self.frame.Show(True)
	
	def export(self, fileName):
		slices = self.sliceSet.slices;
		logging.info("Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...");

		gcodewriter = Gcode_Lib.GCode_Generator();
		gcodewriter.start();
		exportedPaths = 0;
		print "Exporting ",len(slices), " slices."
		for slice in slices:
			gcodewriter.comment('zLevel' + str(slice.zLevel) );
			print "slice has ",slice.paths.Extent()," paths."
			it = TopoDS.TopoDS_ListIteratorOfListOfShape(slice.paths);
			it.Initialize(slice.paths);

			while it.More():
				shape = it.Value();
				
				#the shape could be a wire or a compound.				
				if shape.ShapeType() == TopAbs.TopAbs_WIRE:
					gcodewriter.comment("begin wire");
					wire = ts.Wire(shape);
					exportedPaths += 1;
					#fixWire2(ts.Wire(shape));
					print "Before Sort"
					summarizeWire(wire);
					
					fixed = fixWire(wire);
					print "After Sort"
					summarizeWire(fixed);
					self.frame.showShape(fixed);
					gcodewriter.followWire(fixed,self.feedRate);
					
					#fixWire2(wire);
					#for e in fixWire(wire):
					#	gcodewriter.followEdge(ts.Edge(e),self.feedRate);
					#edges = fixWire2(ts.Wire(shape));
					#for i in range(1,edges.Length()+1):
					#	gcodewriter.followEdge(ts.Edge(edges.Value(i)),self.feedRate);
					#print "HERE!"
					#for e in edges:
					#	gcodewriter.followEdge(e,self.feedRate);
					gcodewriter.comment("end wire");
				elif shape.ShapeType() == TopAbs.TopAbs_COMPOUND:
					gcodewriter.comment("begin compound");
					print "exorting compound path"
					texp = TopExp.TopExp_Explorer();
					texp.Init(shape,TopAbs.TopAbs_WIRE);

					sf = ShapeAnalysis_FreeBounds();
					
					#add all the wires to a sorter.
					while ( texp.More() ):
						wire = ts.Wire(texp.Current());
						exportedPaths += 1;
						gcodewriter.comment("begin wire");
						#fixWire2(ts.Wire(wire));
						#summarizeWire(wire);
						#fixWire2(wire);
						print "Before Sort"
						summarizeWire(wire);
						fixed = fixWire(wire);
						print "After Sort:"
						summarizeWire(fixed);
						self.frame.showShape(fixed);
						gcodewriter.followWire(fixed,self.feedRate);						
						#for e in fixWire(wire):
						#	gcodewriter.followEdge(ts.Edge(e),self.feedRate);
						gcodewriter.comment("end wire");
						#edges = fixWire2(wir);
						#for e in edges:
						#	gcodewriter.followEdge(e,self.feedRate);						
						#edges = fixWire2(ts.Wire(shape));
						#edges = fixWire2(wire);
						#for i in range(1,edges.Length()+1):
						#	gcodewriter.followEdge(ts.Edge(edges.Value(i)),self.feedRate);					
						#gcodewriter.comment("end wire");
						#inputwires.Append(wire);
						print "exported wire of compound"
						texp.Next();
					gcodewriter.comment("end compound");
					#now sort them
					#print "There are ",inputwires.Length()," wires."
					#sf.ConnectWiresToWires(inputwires.GetHandle(),0.0001,False,outputwires.GetHandle() );
					#print "New list has",outputwires.Length()," wires."
					
					##now follow the wires
					#for i in range(1,outputwires.Length()):
					#	gcodewriter.followWire(outputwires.Value(i));
					
				else:
					print "Unknown type of object received in the slice list"
				it.Next();
		commands = gcodewriter.getResults();
		f = open(fileName,'w');
		f.write("\n".join(commands));
		f.close()
		print exportedPaths," paths were exported."
"""
    Writes a sliceset to specified file in SVG format.
"""
class SVGExporter ( ):
	def __init__(self,sliceSet):
		self.sliceSet = sliceSet
		self.title="Untitled";
		self.description="No Description"
		self.unitScale = 3.7;
		self.units = sliceSet.analyzer.guessUnitOfMeasure();
		self.NUMBERFORMAT = '%0.3f';
		
	def export(self, fileName):
		#export svg
		#the list of layers requires a thin layer around the
		#slices in the slice set, due to the transformations required
		#for the fancy viewer
		
		slices = self.sliceSet.slices;
		logging.info("Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...");
		layers = []
		for s in	self.sliceSet.slices:
			layers.append( SVGLayer(s,self.unitScale,self.NUMBERFORMAT) );
				
		#use a cheetah template to populate the data
		#unfortunately most numbers must be formatted to particular precision		
		#most values are redundant from  sliceSet, but are repeated to allow
		#different formatting without modifying underlying data
		
		t = Template(file='svg_template.tmpl');
		t.sliceSet = self.sliceSet;
		t.layers = layers;
		t.units=self.units;
		t.unitScale = self.unitScale;
		
		#adjust precision of the limits to 4 decimals
		#this converts to a string, but that's ok since we're using it 
		# to populate a template
		t.sliceHeight = self.NUMBERFORMAT % t.sliceSet.sliceHeight;
		t.xMin = self.NUMBERFORMAT % t.sliceSet.analyzer.xMin;
		t.xMax = self.NUMBERFORMAT % t.sliceSet.analyzer.xMax;
		t.xRange = self.NUMBERFORMAT % t.sliceSet.analyzer.xDim;
		t.yMin = self.NUMBERFORMAT % t.sliceSet.analyzer.yMin;
		t.yMax = self.NUMBERFORMAT % t.sliceSet.analyzer.yMax;
		t.yRange = self.NUMBERFORMAT % t.sliceSet.analyzer.yDim;
		t.zMin = self.NUMBERFORMAT % t.sliceSet.analyzer.zMin;
		t.zMax =	self.NUMBERFORMAT % t.sliceSet.analyzer.zMax;	
		t.zRange = self.NUMBERFORMAT % t.sliceSet.analyzer.zDim;
		


		#svg specific properties
		t.xTranslate=(-1)*t.sliceSet.analyzer.xMin
		t.yTranslate=(-1)*t.sliceSet.analyzer.yMin
		t.title=self.title
		t.desc=self.description
		
		#put layer dims as nicely formatted numbers
		t.xMinText = "%0.3f" % t.sliceSet.analyzer.xMin; 
		t.xMaxText = "%0.3f" % t.sliceSet.analyzer.xMax;
		t.yMinText = "%0.3f" % t.sliceSet.analyzer.yMin; 
		t.yMaxText = "%0.3f" % t.sliceSet.analyzer.yMax;
		t.zMinText = "%0.3f" % t.sliceSet.analyzer.zMin;
		t.zMaxText = "%0.3f" % t.sliceSet.analyzer.zMax;
		f = open(fileName,'w');
		f.write(str(t));
		f.close()

"""
	One slice in a set of slices that make up a part
"""
class Slice:
	def __init__(self):
		logging.debug("Creating New Slice...");
		self.path = "";
		self.faces = [];
		self.paths = TopoDS.TopoDS_ListOfShape();
		self.zLevel=0;
		self.zHeight = 0;
		self.layerNo = 0;
		self.sliceHeight=0;
		
	def addPath(self, pathShape):
		self.paths.Append(pathShape);
		print "slice at zlevel",self.zLevel,"  now has ",self.paths.Extent()," paths."


	def addFace(self, face ,saveFaceCopy=True):

		if saveFaceCopy:
			copier = BRepBuilderAPI.BRepBuilderAPI_Copy(face);
			self.faces.append(copier.Shape());
			copier.Delete();
				
		ow = brt.OuterWire(face);
		logging.debug(  "Adding OuterWire..." );
		self.addWire(ow);
		
		logging.debug(  "Adding Other Wires..." );
		#now get the other wires
		te = TopExp.TopExp_Explorer();
		
		te.Init(face,TopAbs.TopAbs_WIRE);
		while te.More():
			w = ts.Wire(te.Current());
			if not w.IsSame(ow):
				self.addWire(w);
			te.Next();
		te.Clear();	
		te.Destroy();
		
	def addWire(self, wire):
		logging.debug( "Adding Wire:" + str(wire));
		bwe = BRepTools.BRepTools_WireExplorer(wire);
		while bwe.More():
			edge = bwe.Current();
			self.addEdge(edge);
			bwe.Next();
		bwe.Clear();
	
		
	def addEdge(self, edge):
		range = btool.Range(edge);
		logging.debug( "Edge Bounds:" + str(range) );
		hc= btool.Curve(edge);
		ad = GeomAdaptor.GeomAdaptor_Curve(hc[0]);
	
		#this could be simplified-- QuasiUnformDeflection probably handles a line
		#correcly anyway?
		logging.debug(  "Edge is a curve of type:" + str(ad.GetType()));
		gc = GCPnts.GCPnts_QuasiUniformDeflection(ad,0.1,hc[1],hc[2]);
		i=1;
		numPts = gc.NbPoints();
		logging.debug( "Discretized Curve has" + str(numPts) + " points." );
		while i<=numPts:
			if edge.Orientation() == TopAbs.TopAbs_FORWARD:
				tPt = gc.Value(i);
			else:
				tPt = gc.Value(numPts-i+1);
			i+=1;
			loop.addPoint(tPt.X(),tPt.Y());
	
"""
	Testing Methods/Functions


"""	
def fillFaceGrid(face):
	print "Filling Face Grid..";
	PRECISION = .1;
	TOLERANCE = 0.001;
	grid = fillLib.BooleanGrid(PRECISION);
	
	#get the bounds of the face in x-y space
	box = Bnd.Bnd_Box();
	b = BRepBndLib.BRepBndLib();
	b.Add(face,box);
	
	bounds = box.Get();
	xMin = bounds[0];
	xMax = bounds[3];
	yMin = bounds[1];
	yMax = bounds[4];
	zMin = bounds[2];
	zMax = bounds[5];
	print xMin,xMax,yMin,yMax;
	#assert 1==2
	t2 = Timer();
	#for the entire space, crate a grid based on membership on the face.
	estimatedops = round((xMax-xMin)*(yMax-yMin)/PRECISION/PRECISION)
	operations=0;
	print "There should be at most %d pixels." % estimatedops
	print "Estimated time is %d seconds" % (estimatedops / 2150)
	cX=xMin;
	while cX < xMax:
		cY = yMin;
		while cY<yMax:
			#is the point on the face?
			p = gp.gp_Pnt(cX,cY,zMin);
			zDir = gp.gp().DZ();
			#make a zdirection line at point cX,cY
			#print "computing",cX,cY
			line = gp.gp_Lin( p, zDir  );
			isector = IntCurvesFace.IntCurvesFace_ShapeIntersector();
			isector.Load(face,TOLERANCE);
			isector.Perform(line,-999,9999);
			if isector.IsDone() > 0:
				#print "Line intersects!";
				grid.setPoint(cX,cY,False);
			#else:
				#print "Line does not intersect!";			
			operations += 1;
			cY += PRECISION;
		cX += PRECISION;
	print "Face Grid Fill Done,", len(grid.pixels)," pixels."
	print "Performance: %0.4f ops/second." % (  operations/ t2.elapsed() );
	#print grid.pixels;
	assert 1==2,"I want to stop here"

	
"""
	Create an array of lines that will cover the entire area of a face.
"""
def createFillLines(xMin,xMax,yMin,yMax,spacing, zLevel,angle):
	"angle is the angle from x axis"
	global mainDisplay;
	t = Timer();
	lineDir = gp.gp_Dir( 1,math.cos(math.radians(angle)),zLevel );
	angleRads = math.radians(angle);
	xSpacing = spacing / math.cos(angleRads);
	ySpacing = spacing/ math.sin(angleRads);
	
	#make lines in two sets: one set moving positive along the x axis,
	#one set moving positive along the y axis
	
	#todo: this algo makes lines that are bit too long in some cases, but that's ok i think.
	#todo: it is actually possible to create all of these lines only one time, and then 
	#re-use them for each slice. the bounding box will not change during slicing
	lines = [];
	for xN in frange6(xMin,xMax,xSpacing):
		p1 = gp.gp_Pnt(xN,yMin,zLevel);
		p2 = gp.gp_Pnt(xMax, (xMax - xN)* math.tan(angleRads), zLevel);
		builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2);
		builder.Build();
		#mainDisplay.showShape(builder.Shape());
		lines.append(builder.Shape() );
	#dont duplicate the middle one.
	for yN in frange6(yMin+ySpacing,yMax,ySpacing):
		p1 = gp.gp_Pnt(xMin,yN,zLevel);
		p2 = gp.gp_Pnt((yMax - yN)* math.tan(angleRads), yMax, zLevel);
		builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(p1,p2);
		builder.Build();
		#mainDisplay.showShape(builder.Shape());		
		lines.append(builder.Shape() );	
	
	print "Create Fillings took %d seconds" % t.elapsed();
	return lines;	
	
	
	
"""
   Older Hatching Code

"""
		
		boundaryEdgeParameters = []; #list of lists: each entry is a parameter list that matches the edge
		for curve in boundaryCurves:
			parameters = [];
			boundaryEdgeParameters.append(parameters);
			for point in boundaryIntersections:
				j+=1;
				a = findParameterOnCurve(point,curve); #returns [point, parameter ] tuple
				if  a != None:
					parameters.append(a);
		
			parameters.sort(cmp= lambda x,y: cmp(x[1],y[1])); #sort by parameter
			
		print "Computed %d projections in %0.5f seconds" % (  j, t.elapsed() );

		#now loop through and build boundary edges
		boundaryEdges = [];

		for k in range(0,len(boundaryCurves)):
			curve = boundaryCurves[k];
			params  = boundaryEdgeParameters[k]; #boundaryCurves and boundaryEdgeParameters are the same length
			
			for pN in range(1,len(params)):
				p2 = params[pN][1];
				#print p2;
				p1 = params[pN-1][1];
				edge = edgeFromTwoPointsOnCurve(curve,p1,p2);
				if edge:
					boundaryEdges.append( edge);

			#also create one edge from the last point to the start point
			#boundaryEdges.append(edgeFromTwoPointsOnCurve(curve,params[0][1],params[len(params)-1][1]));
		print "Created %d Boundary Edges." % len(boundaryEdges);

		
class EdgeFollower:
	"returns edges in head-to-tail order, based on which edge is closest"
	"edges are returned so that their orientation is correct"
	"if a starting point is not defined, the first point of the first"
	"edge is used"
	def __init__(self,allEdges,startingPoint=None):
		self.edges  = [];
		self.startingPoint = startingPoint;
		self.edges.extend( allEdges );	
		self.lastEdge = None;
		
		if startingPoint:
			self.startingPoint = startingPoint;
		else:
			if len(allEdges) > 0:
				self.startingPoint = EdgeWrapper(allEdges[0]).firstPoint;
			else:
				self.startingPoint = None;
		
	def _findClosest(self,point):
		"find the edge having a point closest to the provided point"
		"also ensures that the selected edge is oriented correctly"
		if len(self.edges) == 0:
			return None;
			
		bestDistance = None;
		bestEdge = None;
		bestEdgeReversed = False;
		for edge in self.edges:
			ew = EdgeWrapper(edge);
			#initialize the first time through
			if bestDistance == None:
				bestDistance = point.Distance(ew.firstPoint);
				bestEdge = edge;
			
			d = point.Distance(ew.firstPoint);
			if d < bestDistance:
				bestDistance = d;
				bestEdge = edge;
				bestEdgeReversed = False;
				
			d = point.Distance(ew.lastPoint);	
			if d < bestDistance:
				bestDistance = d;
				bestEdge = edge;
				bestEdgeReversed = True;
		
		self.edges.remove(bestEdge);
		if bestEdgeReversed:
			print "Found a match but edge must be reversed."
			return ts.Edge(bestEdge.Reversed());
		else:
			print "Found a match"
			return bestEdge;
					
	def nextEdge(self):
		"get the next edge: returns EdgeWrapper for the selected Edge"
		return self.edges.pop(0);
		
		if self.lastEdge:
			ew = EdgeWrapper(self.lastEdge);
			closest = self._findClosest(ew.lastPoint);
		else:
			closest = self._findClosest(self.startingPoint);
		self.lastEdge = closest;
		return closest;

	def hasMoreEdges(self):
		return len(self.edges) > 0;

		bestDistance = None;
		bestEdge = None;
		bestEdgeReversed = False;
		for edge in edgeList:
			ew = EdgeWrapper(edge);
			#initialize the first time through
			if bestDistance == None:
				bestDistance = point.Distance(ew.firstPoint);
				bestEdge = edge;
			
			d = point.Distance(ew.firstPoint);
			if d < bestDistance:
				bestDistance = d;
				bestEdge = edge;
				bestEdgeReversed = False;
				
			d = point.Distance(ew.lastPoint);	
			if d < bestDistance:
				bestDistance = d;
				bestEdge = edge;
				bestEdgeReversed = True;		
	
class MultiWireBuilder:
	"builds wires, watching each edge to construct"
	"multiple wires if necessary"
	def __init__(self):
		self.wires = [];		
		self.lastEdge = None;
		self.tolerance = 0.001;
		self.wireBuilder = None;
		self.startNewWire();
		
	def startNewWire(self):	
		if self.wireBuilder:
			if self.wireBuilder.IsDone():
				#debugShape(self.wireBuilder.Wire());
				self.wires.append(self.wireBuilder.Wire());
			else:
				raise Exception,"Could Not Build New Wire, Error # %d" % self.wireBuilder.Error();
		self.wireBuilder = BRepBuilderAPI.BRepBuilderAPI_MakeWire();
		self.lastEdge = None;
		
	def addEdge(self,edge):
		ew = EdgeWrapper(edge);
		
		#debugShape(edge);
		log.debug ("Adding Edge:" + str(ew) );
		#time.sleep(1);
		if self.lastEdge:
			d = ew.firstPoint.Distance(EdgeWrapper(self.lastEdge).lastPoint);
			if d >= self.tolerance:
				log.warn( "Warning: edge does not match, starting new wire" );
				self.startNewWire();
		
		self.lastEdge = edge;
		self.wireBuilder.Add(edge);
		if  not self.wireBuilder.IsDone():
			raise Exception,"Edge could not be added, Error %d" % self.wireBuilder.Error();
	def getResult(self):
		self.startNewWire();
		return self.wires;
"""
	EdgeWrapper provides additional functions and features
	for an OCC TopoDS_Edge object
"""
class EdgeWrapper:
	def __init__(self,edge):
		self.edge = edge;
		self.reversed = False;
		
		#get the first and last points for the underlying curve
		curve = BRepAdaptor.BRepAdaptor_Curve(edge);		
		p1 = curve.FirstParameter();
		p2 = curve.LastParameter();
		self.curve = curve;
		self.curveType = curve.GetType();
		#get first and last points.
		fP1 = gp.gp_Pnt();
		fP2 = gp.gp_Pnt();
		BRepLProp_CurveTool.Value(curve,p1,fP1 );
		BRepLProp_CurveTool.Value(curve,p2,fP2 );
	
		#set up the endpoints for easy access?
		if edge.Orientation() == TopAbs.TopAbs_REVERSED:
			self.reversed=True;
			self.firstPoint = fP2;
			self.lastPoint = fP1
		else:
			self.firstPoint = fP1;
			self.lastPoint = fP2;

	def isLine(self):
		return self.curveType == 0;
		
	def isCircle(self):
		return self.curveType == 1;
	
	def __str__(self):
		if self.isLine():
			s = "Line:\t";
		elif self.isCircle():
			s  = "Circle:\t";
		else:
			s = "Curve:\t";
		return s + printPoint(self.firstPoint) + "-->" + printPoint(self.lastPoint);

"""
    WireWrapper provides additional functions and features
	for an OCC TopoDS_Wire object
"""
class WireWrapper:
	def __init__(self,wire):
		self.wire = wire;
		
		#build a list of the edges
		self.edgeList = [];
		bwe = BRepTools.BRepTools_WireExplorer(wire);
		while bwe.More():
			edge = bwe.Current();
			self.edgeList.append(EdgeWrapper(edge));
			bwe.Next();
		bwe.Clear();		
	
	#sort the wire using ShapeFix to 
	#connect and re-order them head-to-tail
	#returns a wire for the constructed wire
	#or a reference to this object if there was a problem constructing it
	def getSortedWire(self,precision=0.001):
	
		sw = ShapeFix_Wire();
		sw.Load(self.wire);
		sw.ClosedWireMode = True;
		sw.FixReorderMode = 1;
		sw.FixConnectedMode = 1;
		sw.FixLackingMode = 1;
		sw.FixGaps3dMode = 1;
		sw.FixConnectedMode = 1;
		sw.FixDegeneratedMode = 1;

		
		if sw.Perform():
			#log.info("WireSorting Was Successful!");
			return WireWrapper(sw.Wire());	
		else:
			#log.info("WireSorting unsuccessful. returning self.");
			return self;

	def __str__(self):
		p = [];
		p.append("Wire (" + str(len(self.edgeList)) + "  edges ):");
		for e in self.edgeList:
			p.append("\t" + str(e) );
		
		return "\n".join(p);
		
		
"""
   Manages a set of loops and points
   a loop is chain of points that end where it begins
   a slice can be composed of multiple faces.
"""
class Loop:
	def __init__(self):
		self.points = [];
		self.pointFormat = "%0.4f %0.4f ";
		self.tolerance = 0.00001;
		
	def addPoint(self,x,y):
		p = gp.gp_Pnt2d(x,y);
		self.points.append( p );
	

	"""
		Print SVG Path String for a loop.
		If the end point and the beginning point are the same,
		the last point is removed and replaced with the SVG path closure, Z
	"""
	def svgPathString(self):
		lastPoint = self.points.pop();		
		if self.points[0].IsEqual( lastPoint,self.tolerance ):
			closed = True;
		else:
			closed = False;
			self.points.append(lastPoint);
		
		s = "M ";
		p = self.points[0];
		s += self.pointFormat % ( p.X(), p.Y() );
		for p in self.points[1:]:
			s += "L ";
			s += self.pointFormat % ( p.X(),p.Y())
		
		if closed:
			s+= " Z";
		
		return s;

		
		"""
    Writes a sliceset to specified file in SVG format.
"""
class SVGExporter ( ):
	def __init__(self,sliceSet):
		self.sliceSet = sliceSet
		self.title="Untitled";
		self.description="No Description"
		self.unitScale = 3.7;
		self.units = sliceSet.analyzer.guessUnitOfMeasure();
		self.NUMBERFORMAT = '%0.3f';
		
	def export(self, fileName):
		#export svg
		#the list of layers requires a thin layer around the
		#slices in the slice set, due to the transformations required
		#for the fancy viewer
		
		slices = self.sliceSet.slices;
		logging.info("Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...");
		layers = []
		for s in	self.sliceSet.slices:
			layers.append( SVGLayer(s,self.unitScale,self.NUMBERFORMAT) );
				
		#use a cheetah template to populate the data
		#unfortunately most numbers must be formatted to particular precision		
		#most values are redundant from  sliceSet, but are repeated to allow
		#different formatting without modifying underlying data
		
		t = Template(file='svg_template.tmpl');
		t.sliceSet = self.sliceSet;
		t.layers = layers;
		t.units=self.units;
		t.unitScale = self.unitScale;
		
		#adjust precision of the limits to 4 decimals
		#this converts to a string, but that's ok since we're using it 
		# to populate a template
		t.sliceHeight = self.NUMBERFORMAT % t.sliceSet.sliceHeight;
		t.xMin = self.NUMBERFORMAT % t.sliceSet.analyzer.xMin;
		t.xMax = self.NUMBERFORMAT % t.sliceSet.analyzer.xMax;
		t.xRange = self.NUMBERFORMAT % t.sliceSet.analyzer.xDim;
		t.yMin = self.NUMBERFORMAT % t.sliceSet.analyzer.yMin;
		t.yMax = self.NUMBERFORMAT % t.sliceSet.analyzer.yMax;
		t.yRange = self.NUMBERFORMAT % t.sliceSet.analyzer.yDim;
		t.zMin = self.NUMBERFORMAT % t.sliceSet.analyzer.zMin;
		t.zMax =	self.NUMBERFORMAT % t.sliceSet.analyzer.zMax;	
		t.zRange = self.NUMBERFORMAT % t.sliceSet.analyzer.zDim;
		


		#svg specific properties
		t.xTranslate=(-1)*t.sliceSet.analyzer.xMin
		t.yTranslate=(-1)*t.sliceSet.analyzer.yMin
		t.title=self.title
		t.desc=self.description
		
		#put layer dims as nicely formatted numbers
		t.xMinText = "%0.3f" % t.sliceSet.analyzer.xMin; 
		t.xMaxText = "%0.3f" % t.sliceSet.analyzer.xMax;
		t.yMinText = "%0.3f" % t.sliceSet.analyzer.yMin; 
		t.yMaxText = "%0.3f" % t.sliceSet.analyzer.yMax;
		t.zMinText = "%0.3f" % t.sliceSet.analyzer.zMin;
		t.zMaxText = "%0.3f" % t.sliceSet.analyzer.zMax;
		f = open(fileName,'w');
		f.write(str(t));
		f.close()		


	def _makeAdapterCurveFromWire(self,wire):
		"Makes a single parameterized adapter curve from a wire"
			
		#make a parameterized approximation of the wire
		adaptor = BRepAdaptor.BRepAdaptor_CompCurve (wire);
		curve = BRepAdaptor.BRepAdaptor_HCompCurve(adaptor);
		curveHandle = curve.GetHandle();
		
		#approximate the curve using a tolerance
		approx = Approx.Approx_Curve3d(curveHandle,0.001,GeomAbs.GeomAbs_C2,200,12);
		if approx.IsDone() and  approx.HasResult():
			# have the result
			anApproximatedCurve=approx.Curve();
			log.debug( "Curve is parameterized between %0.5f and %0.5f " % (  anApproximatedCurve.GetObject().FirstParameter(), anApproximatedCurve.GetObject().LastParameter() ));
			return anApproximatedCurve;
			#builder =  BRepLib.BRepLib_MakeEdge(anApproximatedCurve);
			#self.showShape(builder.Edge());
		else:
			loggin.warn( "Failed to create curve." );
			return None;	
	
def minimumDistanceBetweenShapes(shape1, shape2):
	"returns minimum distance bewteen two shapes"
	bre= BRepExtrema.BRepExtrema_DistShapeShape(shape1,shape2);
	log.debug("Computing Minimum Distance");
	if bre.Perform():
		return bre.Value();
	else:
		return 200;

		
def _makeAdapterCurveFromWire(self,wire):
	"Makes a single parameterized adapter curve from a wire"
		
	#make a parameterized approximation of the wire
	adaptor = BRepAdaptor.BRepAdaptor_CompCurve (wire);
	curve = BRepAdaptor.BRepAdaptor_HCompCurve(adaptor);
	curveHandle = curve.GetHandle();
	
	#approximate the curve using a tolerance
	approx = Approx.Approx_Curve3d(curveHandle,0.001,GeomAbs.GeomAbs_C2,200,12);
	if approx.IsDone() and  approx.HasResult():
		# have the result
		anApproximatedCurve=approx.Curve();
		log.debug( "Curve is parameterized between %0.5f and %0.5f " % (  anApproximatedCurve.GetObject().FirstParameter(), anApproximatedCurve.GetObject().LastParameter() ));
		return anApproximatedCurve;
		#builder =  BRepLib.BRepLib_MakeEdge(anApproximatedCurve);
		#self.showShape(builder.Edge());
	else:
		loggin.warn( "Failed to create curve." );
		return None;		
		
	
#def make_edge(shape):
#    spline = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(shape)
#    spline.Build()
#    return spline.Shape()	
	
#to override sometingselected,
# overrid Viewer3d.    
# def Select(self,X,Y):
#
#		

	def _hatchSlice(self,slice,lastOffset):
		"take the a slice and compute a list of infillWires"
		log.debug( "Hatching Face...." );
		#debugShape(lastOffset);
		#a set of hatch lines that will conver the entire part
		#self.showShape(lastOffset);
		hatchEdges = self._makeHatchLines(lastOffset,slice.zLevel);
		log.debug( "I have %d Hatch Edges" % len(hatchEdges));
				
		#approximate each boundary by a parameterized bspine curve
		boundaryWires = makeWiresFromOffsetShape(lastOffset);
		
		#self.showShape(lastOffset);
		#bm = BoundaryManager();
		#for wire in boundaryWires:
		boundaryIntersections = [];
		#for i in range(1,boundaryWires.Length()+1):
		#	wire = ts.Wire(boundaryWires.Value(i));
			#self.showShape(wire);
			#bm.addWire(wire);
		#	boundaries.append( hatchLib.SegmentedBoundary(

		infillEdges = [];
		boundaryEdges = [];
		
		continueHatching = True;
		reverseEdgeDirection = False;
		boundariesFound = {};
		#for each generated hatch, intersect with each boundary
		for hatchLine in hatchEdges:
			if not continueHatching:
				break;
			else:
				log.debug( "Moving to next Hatch Line");
				
			hatchLineItersectionNodes = [] #all the intersecitons for this single hatchLine
			
			log.debug( "There are %d boundary wires" % boundaryWires.Length());
			#for boundary in boundaryWires:
			for i in range(1,boundaryWires.Length()+1):
				boundary = ts.Wire(boundaryWires.Value(i));
				log.debug("Wire start");
				#debugShape(boundary);
				#time.sleep(.5);
				brp = BRepExtrema.BRepExtrema_DistShapeShape();
				#debugShape(hatchLine);
				#time.sleep(.2);
				brp.LoadS1(boundary);
				brp.LoadS2(hatchLine );

				if brp.Perform() and brp.Value() < 0.001:
					boundariesFound[boundary] = True;
					
					#number of intersections must be even for closed shapes
					#note that we want to avoid lines that are inside of islands.
					pointList = [];

					for k in range(1,brp.NbSolution()+1):
						hatchLineItersectionNodes.append(hatchLib.Node(brp.PointOnShape1(k)));
						#bm.addIntersectionPoint(boundary,brp.PointOnShape1(k));

					for x in hatchLineItersectionNodes:
						log.debug( str(x));
				else:
					log.debug( "No Intersections Found");
			
				#finished with this boundary.
				#build a segmented boundary so we can track all of those points
				boundaryIntersections.append( 
					hatchLib.SegmentedBoundary(boundary,hatchLineItersectionNodes  );
			
			#end for each boundary
			
			if len(hatchLineItersectionNodes) == 0 and (len(boundariesFound) ==  boundaryWires.Length()):
				continueHatching = False;
			
			#sort the points by ascending X, add to list of hashEdges
			hatchLineItersectionNodes.sort(cmp= lambda x,y: cmp(x.X(),y.X()));
			
			#reverse them every other time so that we can follow them in the right order
			#if reverseEdgeDirection:
			#	hatchLineItersectionPoints.reverse();
			#reverseEdgeDirection = not reverseEdgeDirection;
				
			#HACK: dont know how to handle intersections at vertices or
			#tangent lines. for now, just ignore completely these.
			if len(hatchLineItersectionNodes) % 2 == 1:
				print "Detected Odd Number of intersection points. This is ignored for now."
				continue;
				
			#build infillEdges. We do this here so that we know which order to 
			#connect them. We are using to our advantage that the hatch line createse
			#a list of intersection points with increasing x values.
			#we will use lightweight edges to keep math to a minimum
			i = 0;
			while i< len(hatchLineItersectionNodes):
				p1 = hatchLineItersectionNodes[i];
				p2 = hatchLineItersectionNodes[i+1];
				i += 2;
				
				e = hatchLib.NodeEdge(p1,p2);
				if e:
					#debugShape(e);
					#time.sleep(1);
					#debugShape(make_vertex(p1));
					#debugShape(make_vertex(p2));
					infillEdges.append(e);			
					
		#basically, alternate between an internal and and external edges
		if len(infillEdges) == 0:
			log.debug( "No Infill Edges Were Created. Returning" );
			return None;
		log.debug( "There are %d infill edges" % ( len(infillEdges)));

		
		#infill edges is a list of infill Node Edges
		#boundaryIntersections is a list of SegementedBoundaries, having linked lists of nodes
		#both the edges and the SegmentedBoundaries are composed of Node objects, which allow
		#fast lookup of the points as we traverse through the schema
		
		
		#at this point we have a list of edges, sorted in the correct order
		#to traverse. We need to follow them, but alternately follow a boundary
		#edge to connect them together.
		for e in bm.linkEdges(infillEdges):
			debugShape(e);
			#time.sleep(.5);
		#boundaryEdges = bm.buildEdges();		

		#follower = EdgeFollower(infillEdges,boundaryEdges);

		
		#build wires from the edges
		#edgeList  = TopTools.TopTools_HSequenceOfShape();
		#edgeList = [];
		
		#while follower.hasMoreEdges():
		#	e = follower.nextEdge();
			#debugShape(e);
		#	edgeList.append(e );

		#TODO: because of some tolerance issue, i cannot seem to build a wire out
		#of these edges. but that's ok, there's really no reason why we cannot just use a 
		#sequence of edges instead
		#log.info("Building Wires from % Edges..." %  edges.Length() );
		#wireBuilder.ConnectEdgesToWires(edges.GetHandle(),0.01,True,resultWires.GetHandle() );
		#log.info ("Finished: Created %d wires" % resultWires.Length() );
		
		log.info("Finished Following hatching.");
		return edgeList;

		
		
		"""
	Writes a sliceset to specified file in Gcode format.	
	Accepts a slicer, and exports gcode for the slices that were produced by the slicer.
	The slicer has already sliced and filled each layer. This class simply converts
	these toolpaths to Gcode
	
"""		
class GcodeExporter ():
	def __init__(self):

		self.description="Description";
		self.feedRate = 100.0;
		self.numberFormat = "%0.3f";
		self.verbose =False;
		self.display = None;
	
	def export(self, sliceSet, fileName):
		
		slices = sliceSet.slices;
		msg = "Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...";
		log.info("Exporting " + str(len(slices)) + " slices to file'" + fileName + "'...");
		print msg;
		
		gcodewriter = Gcode_Lib.GCode_Generator();
		gcodewriter.verbose = self.verbose;
		gcodewriter.numberFormat = self.numberFormat;
		gcodewriter.start();
		gcodewriter.comment(self.description);
		exportedPaths = 0;

		for slice in slices:
			print "zLevel %0.5f ...." % slice.zLevel,
			gcodewriter.comment('zLevel' + str(slice.zLevel) );
			
			log.info( "zLevel %d, %d offset paths,%d fill edges " % ( slice.zLevel, len(slice.fillWires), len(slice.fillEdges) ));
			gcodewriter.comment("offsets");
			for fw in slice.fillWires:
				if self.verbose:
					gcodewriter.comment("begin wire");
				log.info(">>begin wire");
				#print "base wire:",str(fw);				
				#sw =fw.getSortedWire();
				#print "sorted wire:",str(sw);				
				gcodewriter.followWire(Wire(fw),self.feedRate);
				log.info(">>end wire");
				
				if self.verbose:
					gcodewriter.comment("end wire");				

				#if self.display:
				#	self.display.showShape(fw);
					
				exportedPaths += 1;


			gcodewriter.comment("infill");				
			for e in slice.fillEdges:
				if self.verbose:
					gcodewriter.comment("begin edge");
				gcodewriter.followEdge(Edge(e),self.feedRate);				
				if self.verbose:
					gcodewriter.comment("end edge");
				exportedPaths += 1;
			print "[Done]"
		commands = gcodewriter.getResults();
		f = open(fileName,'w');
		f.write("\n".join(commands));
		f.close()
		print "Gcode Export Complete."
	

######
# Decorates a slice to provide extra computations for SVG Presentation.
# Needed only for SVG display
######	
class SVGLayer:
	def __init__(self,slice,unitScale,numformat):
		self.slice = slice;
		self.margin = 20;
		self.unitScale = unitScale;
		self.NUMBERFORMAT = numformat;
	def zLevel(self):
		return self.NUMBERFORMAT % self.slice.zLevel;
		
	def xTransform(self):
		return self.margin;
		
	def yTransform(self):
		return (self.slice.layerNo + 1 ) * (self.margin + ( self.slice.sliceHeight * self.unitScale )) + ( self.slice.layerNo * 20 );
		
	def svgPathString(self):
		"return svg path string for a slice"

def mesh(shape,event=None):

    # Create the Mesh
    aMeshGen = SMESH.SMESH_Gen()
    aMesh = aMeshGen.CreateMesh(0,True)
    # 1D
    an1DHypothesis = StdMeshers.StdMeshers_Arithmetic1D(0,0,aMeshGen)#discretization of the wire
    #an1DHypothesis.SetLength(0.1);
    an1DHypothesis.SetLength(0.5,False) #the smallest distance between 2 points
    an1DHypothesis.SetLength(2.,True) 	
    #an1DHypothesis.SetLength(0.5,False) #the smallest distance between 2 points
    #an1DHypothesis.SetLength(1.0,True) # the longest distance between 2 points
    #an1DHypothesis.SetDeflection(0.5);
    #an1DHypothessis.SetLength(0.5);
    #an1DAlgo = StdMeshers.StdMeshers_Regular_1D(1,0,aMeshGen) # interpolation
    an1DAlgo = StdMeshers.StdMeshers_Regular_1D(1,0,aMeshGen);
    # 2D
    #a2dHypothseis = StdMeshers.StdMeshers_TrianglePreference(2,0,aMeshGen) #define the boundary
    a2dHypothseis = StdMeshers.StdMeshers_QuadranglePreference(2,0,aMeshGen) #define the boundary	
    a2dAlgo = StdMeshers.StdMeshers_Quadrangle_2D(3,0,aMeshGen)
    #a2dAlgo = StdMeshers.StdMeshers_MEFISTO_2D(3,0,aMeshGen)	
    #a2dAlgo = StdMeshers.StdMeshers_ProjectionSource2D(3,0,aMeshGen)
    #a2dAlgo = StdMeshers.StdMeshers_Projection_2D(3,0,aMeshGen)	
    #a2dAlgo = StdMeshers.StdMeshers_UseExisting_2D(3,0,aMeshGen)

    #Calculate mesh
    aMesh.ShapeToMesh(shape)
    #Assign hyptothesis to mesh
    aMesh.AddHypothesis(shape,0)
    aMesh.AddHypothesis(shape,1)
    aMesh.AddHypothesis(shape,2)
    aMesh.AddHypothesis(shape,3)
    #Compute the data
    aMeshGen.Compute(aMesh,aMesh.GetShapeToMesh())
    return aMesh;
	
def display_mesh(display, the_mesh):
    # First, erase all
    #display.EraseAll()
    # then redisplay the shape
    #display.showShape(aShape)
    # then the mesh
    aDS = SMESH.SMESH_MeshVSLink(the_mesh)
    aMeshVS = MeshVS.MeshVS_Mesh(True)
    DMF = 1 # to wrap!
    MeshVS_BP_Mesh       =  5 # To wrap!
    aPrsBuilder = MeshVS.MeshVS_MeshPrsBuilder(aMeshVS.GetHandle(),DMF,aDS.GetHandle(),0,MeshVS_BP_Mesh)
    aMeshVS.SetDataSource(aDS.GetHandle())
    aMeshVS.AddBuilder(aPrsBuilder.GetHandle(),True)
    #Create the graphic window and display the mesh
    context = display.canva._display.Context
    context.Display(aMeshVS.GetHandle())
    context.Deactivate(aMeshVS.GetHandle())	
	
	
	
def pause():
	raw_input("Press Enter to continue");
	
#TODO: Vertex Nodes and Inodes need to be different classes extending Node
class Node:
	"represents a point on a boundary, with two references to the next and previous nodes"
	"the use of a wrapper class also allows the nodes to be moved slightly if needed, without"
	"affecting the ability to very quickly find the node using hash lookups"
	
	def __init__(self,point,isVertex):
	
		"these are used for all nodes"
		self.point = point;
		self.boundary = None; #the boundary this node lies upon		
		self.nextNode = None; #next node along the boundary, wrapping around
		self.prevNode = None; #previous node along the boundary, wrapping around
		self.vertex = isVertex; # a vertex is a node that is there because the curve changes along the wire
		self.used = False;

		
		#these only used for intersection ( not vertex ) nodes
		self.infillNode = None; #the other node that makes up the edge of this hatch line

		#for vertex nodes these values will be different, because the edges on eitherside
		#of the node are different. But for intersection nodes, the values will always match
		self.prevEdge = None;
		self.nextEdge = None;
		self.paramOnPrevEdge = None;
		self.paramOnNextEdge = None;
		
	def canMoveInfill(self):
		return self.infillNode and (not self.infillNode.used);
	
		
	@staticmethod
	def toString(node):
		if node.vertex:
			#return "Vertex(%d), Used = %s" % ( id(node),node.used );
			return "Vertex(%d), Used= %s, prevEdge= %d, nextEdge=%d,loc=[%0.2f, %0.2f, %0.2f]"  %  ( id(node), node.used, hashE(node.prevEdge.edgeWrapper.edge), hashE(node.nextEdge.edgeWrapper.edge),node.point.X(), node.point.Y(), node.point.Z() );
		else:
			#return "INode(%d), Used = %s" % ( id(node),node.used );
			return "INode(%d), Used= %s, prevEdge= %d, nextEdge=%d, P1=%0.2f,P2=%0.2f, loc=[%0.2f, %0.2f, %0.2f]"  %  ( id(node), node.used, hashE(node.prevEdge.edgeWrapper.edge), hashE(node.nextEdge.edgeWrapper.edge),node.paramOnPrevEdge,node.paramOnNextEdge, node.point.X(), node.point.Y(), node.point.Z() );

	def __str__(self):
		
		s =  Node.toString(self);
		if self.nextNode:
			s += "\n\t\tNext-->" + Node.toString(self.nextNode);
		if self.prevNode:
			s += "\n\t\tPrev-->" + Node.toString(self.prevNode);
		if self.infillNode:
			s += "\n\t\tinFill-->" + Node.toString(self.infillNode);
		return s;

def findNextInfillNode(node):
	"finds the next infill node in either direction"
	n = findNextInfillNodeForward(node.nextNode,[node]);
	if n:
		return n;
	log.debug("could not find next node");
	
	n = findNextInfillNodeBackward(node.prevNode,[node]);
	if n:
		return n;
	
	return None;
	
def findNextInfillNodeForward(node,path=[]):
	"finds the next infill node in the forward direction, starting with supplied node"

	path = path +[node];
	if node.used:
		return None;
		
	if node.vertex:
		log.debug("node is a vertex, deferring");
		if node.nextNode:
			return findNextInfillNodeForward(node.nextNode,path);
	else:
		if node.canMoveInfill():
			return path ;
			
	return None;		

def findNextInfillNodeBackward(node,path=[]):
	"finds the next infill node in the forward direction, starting with supplied node"
	path = path +[node];
	if node.used:
		return None;
		
	if node.vertex:
		log.debug("node is a vertex, deferring");
		if node.prevNode:
			return  findNextInfillNodeBackward(node.prevNode,path);
	else:
		if node.canMoveInfill():
			return path;
			
	return None;	

def isCloserThan(shape1,shape2,distance):

	brp = BRepExtrema.BRepExtrema_DistShapeShape();
	brp.LoadS1(shape1);
	brp.LoadS2(shape2);
	
	if brp.Perform():
		if brp.Value() < distance:
			#TestDisplay.display.eraseAll();
			#TestDisplay.display.showShape(shape1);
			#TestDisplay.display.showShape(shape2);
			log.warn ("Distance is %0.2f" % brp.Value() );
			#time.sleep(0.2);
		return brp.Value() < distance;
	else:
		log.warn("Trouble Computing Closest Distance!");
		return False;

	
class BoundaryEdge:
	"represents a boundary edge. Composese a regular edge"
	def __init__ ( self,edge):
		self.edgeWrapper = Wrappers.Edge(edge);
		#TestDisplay.display.showShape(edge);
		self.id = hashE(edge);
		self.startNode = None;
		self.endNode = None;
		self.middleNodes = [];
		
	def __str__(self):
		return "BoundaryEdge:\n  Start Node:%s \n  EndNode:%s" % ( str(self.startNode),str(self.endNode));
	
	def __repr__(self):
		return self.__str__();
		
def linkNodes(nodeList,wrapAround=False):
	"link all of the nodes in the list head-to-tail"
	
	for i in range(1,len(nodeList)):
		n1 = nodeList[i-1];
		n2 = nodeList[i];
		
		n1.nextNode = n2;
		n2.prevNode = n1;
		
	if wrapAround:
		n1 = nodeList[0];
		n2 = nodeList[len(nodeList)-1];
		n1.prevNode = n2;
		n2.nextNode = n1;
		

def hashE(edge):
	return edge.HashCode(1000000);

class SegmentedBoundary:
	"""
		represents a boundary.
		on construction the boundary will map its edges,
		
		The object can then provide wires that link nodes along its edges		
	"""

	def __init__(self,wire):

		self.wire = wire;

		self.tolerance = 0.000001;
		self.edgeHashMap = {};	 # a hash of boundary edge objects
		#self.edges = [] #needed to store the lists in the original, wire order
		self.edges = [];
		
		self.fingers = []; #normal edges to this edge, used to prevent intersections with hatch lines
		
		#build a graph of the edges on the boundary
		#we'll use this to build edges between nodes later on
		log.debug("Building Edge Graph...");
		wr = Wrappers.Wire(wire);
		
		edgeSeq = wr.edgesAsSequence();
		for i in range(1,edgeSeq.Length()+1):
			edge = Wrappers.cast(edgeSeq.Value(i));
			self.addEdge(edge);

	
		log.debug("Finished Adding Edges.");
		#build nodes between the edges
		#traverse the edges forwards to create start nodes
		previousNode = None;
		previousEdge = None;
		
		for be in self.edges:
			log.debug("Procssing Edge %s" % be.id);
			#create a node at the start of the edge
			#this node is a vertex, it has a parameter on two edges
			node = Node(be.edgeWrapper.firstPoint,True);
			node.boundary = self;
			node.nextEdge = be;
			node.paramOnNextEdge = be.edgeWrapper.firstParameter;
			be.startNode = node;
			
			if previousNode:
				previousEdge.endNode = node;
				node.prevEdge = previousEdge;
				node.paramOnPrevEdge = previousEdge.edgeWrapper.lastParameter;
				
				#connect the nodes together
				previousNode.nextNode = node;
				node.prevNode = previousNode;
				
			#save current values for next pass through the loop
			previousNode = node;
			previousEdge = be;
		
		#now all are linked but the last node. of the last edge, which is 
		#handled differently depending on if the wire is closed or not
		firstNode = self.edges[0].startNode;
		lastEdge = self.edges[len(self.edges)-1];
		
		if wire.Closed():
			log.debug("Wire Is Closed, linking end to start");
			lastEdge.endNode = firstNode;
			lastEdge.startNode.nextNode = firstNode;
			firstNode.prevEdge = lastEdge;
			firstNode.paramOnPrevEdge = lastEdge.edgeWrapper.lastParameter;
			firstNode.prevNode = lastEdge.startNode;
			
		else:
			#last edge gets a new node
			node = Node(previousEdgeW.edgeWrapper.lastPoint,True);
			node.boundary = self;
			node.prevEdge = lastEdge;
			lastEdge.endNode = node;

			firstNode.prevNode = node;
			node.nextNode = firstNode;
		
		
	def addEdge(self,edge):
		"add an edge to the boundary"
		be = BoundaryEdge(edge);
		self.edges.append(be);
		self.edgeHashMap[be.id] = be;
			
	def newIntersectionNode(self, edge,edgeParam, point):
		"returns a new intersection node on this boundary."
		"the boundary will adjust neighbors to work it into the boundary"
	
		be = self.edgeHashMap[hashE(edge)];
		#make a new node to insert into the list
		newNode = Node(point,False); #an intersection point
		
		newNode.boundary = self;
		
		#bit of a hack-- a node in the middle of an edge
		#gets simulated values for next and previous
		#edges, just like a vertex. that makes making edges easier later.
		#TestDisplay.display.showShape( Wrappers.make_vertex(point));
		#log.warn("Point Parmeter is %0.2f"% edgeParam);
		#time.sleep(3);
		
		newNode.paramOnPrevEdge = edgeParam;
		newNode.paramOnNextEdge = edgeParam;
		newNode.prevEdge = be;
		newNode.nextEdge = be;
			
		be.middleNodes.append(newNode);	
		#choice of paramOnPrevEdge or paramOnNextEdge is arbirary: these nodes
		#are all Intersections on the same edge, so paramOnPrevEdge always= paramOnNextEdge 
		be.middleNodes.sort(cmp= lambda x,y: cmp(x.paramOnPrevEdge,y.paramOnPrevEdge));
		
		#check the parameters to make sure to sort the middle nodes correctly
		if be.endNode.paramOnPrevEdge < be.startNode.paramOnNextEdge:
			be.middleNodes.reverse();
	
		#now link the nodes together
		tmpList = [];
		tmpList.append(be.startNode);
		tmpList.extend(be.middleNodes);
		tmpList.append(be.endNode);
		linkNodes(tmpList,False);
		
		return newNode;
		
	def edgesToInFillNode(self,startNode):
		"returns a list of edges and the node that should be used next"
		"if no next code can be found, None is returned"
		assert startNode.boundary==self,"Expected to get a startnode on my boundary";		
		
		#search the node graph. if a vertex is encountered, it means we'll have to add an edge
		pathToNextInfill = findNextInfillNode(startNode);
		
		if pathToNextInfill == None:
			log.info("Could not find valid path to the next infill");
			return None;
			
		log.debug("Path has %d Nodes In it" % len(pathToNextInfill) );
		
		#print "***** NODE PATH *****"
		#for n in pathToNextInfill:
		#	print str(n);
		#print "**** END NODE PATH****"
		assert len(pathToNextInfill ) >1,"Expected a Path with at least two nodes"		
		edgesToReturn = [];
		
		#ok we have a path, starting with startNode and ending
		#with a node that is an infill node
		edgesToReturn = []; #the edges that will become our return list
		
		for (startNode,endNode) in ntuples(pathToNextInfill,2,False):  #iterate over the list in pairs of nodes
			#be careful, we might be moving along the edge opposite to its original direction
			#ie, nextEdge and prevEdge might be reversed
			#TODO: is there a cleaner way to do this?
			newEdge = None;
			if startNode.nextEdge == endNode.prevEdge:
				newEdge = startNode.nextEdge.edgeWrapper.trimmedEdge( startNode.paramOnNextEdge,endNode.paramOnPrevEdge);
			elif startNode.nextEdge == endNode.nextEdge:
				newEdge = startNode.nextEdge.edgeWrapper.trimmedEdge( startNode.paramOnNextEdge,endNode.paramOnNextEdge);
			elif startNode.prevEdge == endNode.prevEdge:
				newEdge = startNode.prevEdge.edgeWrapper.trimmedEdge( startNode.paramOnPrevEdge,endNode.paramOnPrevEdge);
			elif startNode.prevEdge == endNode.nextEdge:
				newEdge = startNode.prevEdge.edgeWrapper.trimmedEdge( startNode.paramOnPrevEdge,endNode.paramOnNextEdge);
			else:
				raise ValueError,"Two consecutive nodes did not share an edge!"
				
			edgesToReturn.append(newEdge );

			startNode.used = True;
			endNode.used = True;
		
		log.info("Path Contains %d Edges." % len(edgesToReturn) );
		lastNode = pathToNextInfill.pop();
		return  [edgesToReturn,lastNode];

			def edges(self):
		log.debug("Following Node Map...");				
		nodesLeft = Hatcher.findUnTracedNodes(self.allIntersectionNodes );
		numNodesLeft = len(nodesLeft);
		
		if numNodesLeft == 0:
			log.warn("No Intersection Nodes were found!");
			return;
			
		currentNode = nodesLeft[0];
		findingInfillEdge = True; #used to alternate between boundary and infill edges
		
		while ( numNodesLeft > 0):

			currentNode.used = True;
			numNodesLeft -= 1;

			if findingInfillEdge:
				log.debug("looking for an infill edge..");
				#find an infill edge
				if currentNode.canMoveInfill():
					
					#return edge
					newNode = currentNode.infillNode
					currentNode.used = True;
					newNode.used = True;
					#TestDisplay.display.showShape(Wrappers.make_vertex(newNode.point));
					#print "***** NODE PATH *****"
					#print str(currentNode)
					#print str(newNode);
					#print "***** END NODE PATH ****"
					candidateEdge = Hatcher.linearEdgeBetweenNodes(currentNode,newNode);
					#cew = Wrappers.Edge(candidateEdge);
					#TRIMLENGTH = 0.6;
					#print "Old P1=%0.2f,P2=%0.2f  " % (cew.firstParameter, cew.lastParameter )
					#shorterEdge = cew.trimmedEdge(cew.firstParameter+ TRIMLENGTH, cew.lastParameter -TRIMLENGTH );
					
					#check to make sure this edge is not too close to other boundaries.
					#if it is, we'll simply supress it 
					#TODO: replace this with better code to build a replacement
					#the general approach is to trim the edge on the ends (so that it does not touch either end )
					#then check the minimum distance against the offset wire. If the distance is too short, we need
					#to throw out this wire
					tooClose = False;

					#for w in self.boundaryWires:
						#dont check the boundary that this line starts and ends on
					#	if shorterEdge:
					#		if isCloserThan(shorterEdge, w, 0.002 ):
					#			#print cew.firstParameter, cew.lastParameter;
					#			tooClose = True;
					#			break;
					
					if not tooClose:
						yield candidateEdge;
					else:
						log.warn("Hatch Line is too close to boundary, skipping it. Later you should create a better edge instead");
						
					currentNode = newNode;					
					findingInfillEdge = False;
				else:
					log.debug("current node does not have an infill path available, finding a new starting node ");
					#must find a new infill edge		
					currentNode = self.findFirstInfillNode();

			else:
				log.debug("looking for a boundary edge..");
				nn = currentNode.boundary.edgesToInFillNode(currentNode);
				if nn:
					#TestDisplay.display.showShape(Wrappers.make_vertex(nn[1].point));
					log.debug("next node is available for a move, using that one");
					for e in nn[0]:
						yield e;
					currentNode = nn[1];
				else:
					log.debug("current node does not have any boundary nodes available, selecting new infill node");
					#need to select a new infill Edge
					currentNode = self.findFirstInfillNode();
						
				findingInfillEdge = True;

		#done-- no nodes left
		log.debug("No More Nodes Left.");
		
	def _makeHatchLinesOld(self):
		"Makes a set of hatch lines that cover the specified shape"
		log.debug("Making Hatch Lines...");
		
		hatchEdges = [];
		#this algo is a simple one that simply does 45 degree hatching
		xMin = self.bounds[0] - ( self.HATCH_PADDING);
		yMin = self.bounds[1] - ( self.HATCH_PADDING);		 
		xMax = self.bounds[2] + (self.HATCH_PADDING);
		yMax = self.bounds[3] + (self.HATCH_PADDING) ;
		dX = xMax  - xMin;
		dY = yMax - yMin;
		maxP = 1.414 * max(dX,dY); # the diagonal of the square. this is our indexing parameter

		
		for p in Wrappers.frange6(0,maxP,self.infillSpacing):				
			#make an edge to intersect
			dl = p * 1.414 #length of a side
			if self.reverse:
				#travelling lower left to upper right
				p1 = gp.gp_Pnt(xMin ,yMin + dl,self.zLevel);
				p2 = gp.gp_Pnt(xMin + dl,yMin, self.zLevel);			
			else:
				#upper left to lower right
				p1 = gp.gp_Pnt(xMin,yMax - dl,self.zLevel);
				p2 = gp.gp_Pnt(xMin + dl,yMax,self.zLevel );

			edge = Wrappers.edgeFromTwoPoints(p1,p2);
			
			if edge:
				#TestDisplay.display.showShape(edge);
				hatchEdges.append(edge);			
	
		
		return hatchEdges;		
		
def checkWalkClosedWire(boundary,forward=True):
	"walk a wire, make sure its possible to get back to the start"
	
	MAX_HOPS = len(boundary.edges)*2;
	firstNode = boundary.edges[0].startNode;
	
	if forward:
		node = firstNode.nextNode;
	else:
		node = firstNode.prevNode;
	numHops = 0;
	
	while node != firstNode and numHops < MAX_HOPS:
		if forward:
			node = node.nextNode;
		else:
			node = node.prevNode;
		numHops += 1;
	log.debug("Traversed Wire in %d hops" % numHops );
	assert numHops < MAX_HOPS,"Too many hops to traverse the wire"

def checkNode(node,desc):
	"checks a node to ensure it looks right"
	assert node != None,desc + "Node is null"
	assert node.nextNode !=None,desc + ":No next node";
	assert node.prevNode !=None,desc +":No prev Node";
	assert node.nextNode != node, desc +":Next is self reference";
	assert node.prevNode != node, desc+":Prev is self reference";
	assert node.prevEdge != None, desc+":No prev edge";
	assert node.nextEdge != None, desc+":No next edge";

def checkBoundary(sb):
	"checks a boundary for correctness"
	print "Segmented boundary for square wire....",
	assert len(sb.edges) == 4, "There should be 4 edges"
	
	#make sure each edge has a start and end node, and each of those
	#has a next and previous node
	for edge  in sb.edges:
		checkNode(edge.startNode,"StartNode");
		checkNode(edge.endNode,"EndNode");
		
	checkWalkClosedWire(sb,True);
	checkWalkClosedWire(sb,False);			
	print "OK"	

		@staticmethod
	def findUnTracedNodes(listOfNodes):
		"finds nodes in the list that have not been traced"
		"a graph is complete if all nodes which have infill connections have been traced"
		un = [];
		for n in listOfNodes:
			if not n.used:
				un.append(n);
		return un;

	def findFirstInfillNode(self):
		"find the first node that can be part of an infill edge"
		for n in self.allIntersectionNodes:
			if n.infillNode and (not n.used) :
				return n;
				
	@staticmethod
	def linearEdgeBetweenNodes(startNode,endNode):
		"make a linear edge from two points "
		builder = BRepBuilderAPI.BRepBuilderAPI_MakeEdge(startNode.point,endNode.point);
		builder.Build();
		if builder.IsDone():
			return builder.Edge();
		else:
			return None;	
