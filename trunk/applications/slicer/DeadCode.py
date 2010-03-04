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
		