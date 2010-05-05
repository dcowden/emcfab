"""
	An exporter that writes svg files.
	Most of the intelligence of following paths is available in PathExport.
	
	This simply translates ArcMove and LinearMove objects into svg syntax
"""
import os,sys,logging,math
from OCC import gp;
from OCC import TopTools
from string import Template
import PathExport
import TestDisplay
import Wrappers
log = logging.getLogger('SVGExporter');

NUMBERFORMAT = '%0.3f';	

class SVGExporter():
	def __init__(self,slicer,options):
		self.slicer = slicer;
		self.title="Untitled";
		self.description="No Description"
		self.unitScale = 3.7;
		#self.unitScale = 1;
		self.units = options.units;
		self.options = options;
		
		self.xTranslate=(-1)*self.slicer.analyzer.xMin
		self.yTranslate=(-1)*self.slicer.analyzer.yMin
	
	def export(self, fileName):
	
		f = open(fileName,'w');

		print "Exporting " + str(len( self.slicer.slices)) + " slices to file'" + fileName + "'...";
		
		#compute the top section
		d = {};
		d.update(self.slicer.analyzer.__dict__.copy() ); #dump in properties of the analyzer
		d.update( self.slicer .__dict__.copy() ); #dump in properties of the slicer
		d.update(self.__dict__.copy() ); #dump in my properties
		d.update(self.options.__dict__.copy() );
		
		f.write( topTemplate.substitute(d));
		
		#for each slice, print the paths
		for s in	self.slicer.slices:
			
			#stash a bunch of properties for conversion			
			ld = {
				'unitScale': self.unitScale,
				'xTransform': 20,
				'margin': 20,
				'yTransform' :0 ,
				'zLevel': ( self.options.svg.numberFormat %( s.zLevel) ),
				'layerNo' : s.layerNo,
				'path': self.computeLayerPath(s )
			};
			ld.update(d);
			f.write(pathTemplate.substitute(ld) );
	
		#compute the bottom section
		f.write(bottomTemplate.substitute(d));
		
		print "SVG Export Complete."
		f.close();
		
	def computeLayerPath(self,slice):
		"computes paths for a single layer"

		pe = PathExport.ShapeDraw(True,0.001 );
		path = [];
		
		allShapes = TopTools.TopTools_HSequenceOfShape();
		Wrappers.extendhSeq( allShapes, slice.fillWires );
		Wrappers.extendhSeq( allShapes, slice.fillEdges );
		for move in pe.follow( allShapes):
			moveType = move.__class__.__name__;
			if moveType == "LinearMove":
				path.append(self.linearMove(move) );
			elif moveType == "ArcMove":
				path.append(self.arcMove(move) );
			else:
				raise ValueError,"Unknown Move Type!"

		return "\n".join(path);		

	
	def linearMove(self,move):
		"get svg path substring from a linear move"
		nf = self.options.svg.numberFormat;
		v = move.vector();
		if move.draw:
			s = "L" + nf + "," + nf;
		else:
			s = "M" + nf + "," + nf;
		return s % (v[0] , v[1]);
	
	
	def arcMove(self,move):
		v = move.vector();
		"get svg path substring from an arc move"
		nf = self.options.svg.numberFormat;
		rx  = ry = move.getRadius();
		rotation = 0;
		x = v[0];
		y = v[1];

		if move.includedAngle > math.pi:
			largeArc = 1;
		else:
			largeArc = 0;
		if move.ccw:
			sweep = 1;
		else:
			sweep = 0;
		# A rx ry xAxisRotation largeArcFlag sweepFlag endX endY
		s  = "A" + nf + "," + nf + ", %d , %d, %d, " +nf + "," + nf;
		return s % ( rx,ry,rotation,largeArc,sweep,x,y) ;
		
		
	

"template for output svg. It is pretty long"
topTemplate = Template("""<?xml version="1.0" standalone="no"?>
<!--
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

-->
<svg contentScriptType="text/ecmascript"
	baseProfile="full" zoomAndPan="magnify" contentStyleType="text/css"
	width="550px" height="1680px" onload="init()"
	preserveAspectRatio="xMidYMid meet"
	xmlns="http://www.w3.org/2000/svg" version="1.0"
	xmlns:xlink="http://www.w3.org/1999/xlink"
	xmlns:slice="http://www.reprap.org/slice">
	<script type="text/ecmascript"><![CDATA[
//Meta data variables 
units='mm';
//End of svg header
sliceMinX = 0;
sliceMaxX = 0;
sliceMinY = 0;
sliceMaxY = 0;
sliceMinZ = 0;
sliceMaxZ = 0;

//Control var's
currentLayer = 0; //Number of currently viewed layer (zero index)
javascriptControlBoxX = 510;
javascriptControlBoxY = 95;
sliding = false;

//Display var's
margin = 20;
scale = 1; //unitScale * zoomScale
sliceDimensionX = null;
sliceDimensionY = null;
sliceDimensionZ = null;
svgMinWidth = javascriptControlBoxX + 2 * margin; //Width of control box and margins
textHeight = 22.5;
unitScale = $unitScale; //mm = 1; inch = 25.4
zoomScale = 7.5; //Default 1:1 may need smaller scale for large objects

//No javascript control var
noJavascriptControlBoxY = 110;

var layers = [];

function changeScale(newScale){
	zoomScale = newScale
	scale = unitScale * zoomScale
	viewSingle()
	setText('scaleNum','1 : ' + (1/zoomScale).toFixed(2));
	if(zoomScale >=1 ) //dont scale line thickness for large display scale 
		document.getElementById('layerData').setAttributeNS(null,'stroke-width',2/(scale))
}

function displayLayer(layerNum){
	if (layerNum >= 0 && layerNum < layers.length) {
		layers[currentLayer].setAttributeNS(null,'visibility','hidden');
		layers[layerNum].setAttributeNS(null,'visibility','visible');
		currentLayer = layerNum;
		setText('layerNum',currentLayer+1);
		//Slider
		if (!sliding) {
			// 150 = width of sliderbar 14 = width circle
			x = (150 - 14) / (layers.length - 1) * currentLayer
			document.getElementById('thumb').setAttributeNS(null,'cx',x + 17)
		}
	}
}

function init(){
	//Hide no javascript control box
	document.getElementById('noJavascriptControls').setAttributeNS(null,'visibility','hidden')
	
	//Get meta data
	mD = document.getElementsByTagNameNS('http://www.reprap.org/slice','layers')[0];
	units = mD.getAttributeNS(null,'units');
	sliceMinX = (mD.getAttribute('minX') * 1).toFixed(2);
	sliceMaxX = (mD.getAttribute('maxX') * 1).toFixed(2);
	sliceMinY = (mD.getAttribute('minY') * 1).toFixed(2);
	sliceMaxY = (mD.getAttribute('maxY') * 1).toFixed(2);
	sliceMinZ = (mD.getAttribute('minZ') * 1).toFixed(2);
	sliceMaxZ = (mD.getAttribute('maxZ') * 1).toFixed(2);
	
	//Set Display variables
	unitScale = units == 'in' ? 96 : 3.7;
	scale = unitScale * zoomScale;
	sliceDimensionX = Math.round((sliceMaxX - sliceMinX) * 1000) /1000; //Rounding to 3 decimal places
	sliceDimensionY = Math.round((sliceMaxY - sliceMinY) * 1000) /1000;
	sliceDimensionZ = Math.round((sliceMaxZ - sliceMinZ) * 1000) /1000;
	
	//Find all groups
	var allGroups = document.getElementsByTagName('g');
	
	//Find only layer groups
	for ( var i = 0;i<allGroups.length;i++){
		if ( allGroups[i].id.indexOf('z') == 0 ){
			layers.push(allGroups[i] );
		}
	}
	
	//Slider
	thumb = document.getElementById('thumb');
	thumb.addEventListener("mousedown", SliderDown, false);
	thumb.addEventListener("mouseup", SliderUp, false);
	thumb.addEventListener("mousemove", SliderMove, false);
	
	//Control box data
	setText('layerMax', layers.length)
	setText('minX', sliceMinX)
	setText('minX', sliceMinX)
	setText('minY', sliceMinY)
	setText('minZ', sliceMinZ)
	setText('maxX', sliceMaxX)
	setText('maxY', sliceMaxY)
	setText('maxZ', sliceMaxZ)
	setText('dimX', sliceDimensionX)
	setText('dimY', sliceDimensionY)
	setText('dimZ', sliceDimensionZ)
	setText('scaleNum','1 : ' + 1/zoomScale);
	setText('layerThickness', 'Layer Thickness: ' + (mD.getAttribute('layerThickness') *1 ).toFixed(2) + units);
	
	changeScale(zoomScale);
}

function setSVG(width, height) {
	svgObjs = document.getElementsByTagName('svg');
	rootSVG = svgObjs[0];
	rootSVG.setAttributeNS(null,'width',width + 'px')
	rootSVG.setAttributeNS(null,'height',height + 'px')
}

function setText(id, str ){
	e = document.getElementById(id)
	if ( e != null )
		e.firstChild.nodeValue = str;
}

function SliderDown(event){sliding = true;}

function SliderUp(event){sliding = false;}

function SliderMove(event){
	value = event.clientX - 130
	thumb = document.getElementById("thumb");
	if (sliding && value > 6 && value < 144){
		thumb.setAttribute("cx", 10 + value);
		zoneWidth = (150 - 14) / (layers.length)
		newLayer = Math.round((value - 7 - 0.5 * zoneWidth) / zoneWidth)
		if(newLayer != currentLayer){
			displayLayer(newLayer)
		}
	}
}

function viewAll(){
	//Set svg size and view port
	width = margin + (sliceDimensionX * unitScale) + margin;
	width = Math.max( width, svgMinWidth );
	height = layers.length * (margin + sliceDimensionY * unitScale + textHeight) + 3 * margin + textHeight + noJavascriptControlBoxY
	setSVG(width,height);
	
	//move and show all layers 
	for (var i in layers) {
		x = margin
		//y = margin + sliceDimensionY * scale
		y = (1 * i + 1) * ( margin + sliceDimensionY * unitScale) + i * textHeight
		layers[i].setAttributeNS(null,'transform','translate(' + x + ', ' + y + ')')
		transform = 'scale(' + unitScale + ' ' + (unitScale * -1) + ') translate(' + (sliceMinX * -1) + ' ' + (sliceMinY * -1) + ')'
		layers[i].getElementsByTagName('path')[0].setAttributeNS(null,'transform',transform)
		layers[i].setAttributeNS(null,'visibility','visible')
		layers[i].getElementsByTagName('text')[0].setAttributeNS(null,'visibility','visible')
	}
	
	//show control box
	document.getElementById('javascriptControls').setAttributeNS(null,'visibility','hidden')
	document.getElementById('noJavascriptControls').setAttributeNS(null,'visibility','hidden')
	x = margin
	y = layers.length * (margin + sliceDimensionY * unitScale + textHeight) + margin
	document.getElementById('buttonSingle').setAttributeNS(null,'visibility','visible')
	document.getElementById('buttonSingle').setAttributeNS(null,'transform','translate('+x+' '+y+')')
}

function viewSingle(){
	//Set svg size and view port
	width = margin + (sliceDimensionX * scale) + margin;
	width = Math.max( width, svgMinWidth );
	height = margin + (sliceDimensionY * scale) + margin + javascriptControlBoxY + margin 
	setSVG(width,height);
	
	//move and hide all layers 
	for (var i in layers) {
		x = margin
		y = margin + sliceDimensionY * scale
		layers[i].setAttributeNS(null,'transform','translate(' + x + ' ' + y + ')')
		layers[i].setAttributeNS(null,'visibility','hidden')
		//layers[i].getElementsByTagName('text')[0].setAttributeNS(null,'visibility','hidden')
		transform = 'scale(' + scale + ' ' + (scale * -1) + ') translate(' + (sliceMinX * -1) + ' ' + (sliceMinY * -1) + ')'
		layers[i].getElementsByTagName('path')[0].setAttributeNS(null,'transform',transform)
	}
	
	//show control box
	document.getElementById('javascriptControls').setAttributeNS(null,'visibility','visible')
	document.getElementById('noJavascriptControls').setAttributeNS(null,'visibility','hidden')
	x = margin
	y = margin + sliceDimensionY * scale + margin
	document.getElementById('javascriptControls').setAttributeNS(null,'transform','translate('+x+' '+y+')')
	document.getElementById('buttonSingle').setAttributeNS(null,'visibility','hidden')
	
	//show current layer
	displayLayer(currentLayer);
}
	]]></script>
	
	<title>$title</title>
	<desc>$description</desc>
	<metadata>
		<slice:layers id="sliceData" version="0.1" units="$units" layerThickness="$layerHeight" 
				minX="$xMin" maxX="$xMax" 
				minY="$yMin" maxY="$yMax" 
				minZ="$zMin" maxZ="$zMax"/>
	</metadata>
	<!--Begin Layer Data   -->
	<g id="layerData" fill="none" stroke="blue" stroke-width="4"  
	font-weight="bold" font-family="Arial" font-size="15px">
<!--Beginning of path section-->
<!-- transform algorithm
	unit scale (mm=3.7, in=96)
	
	g transform
		x = margin
		y = (layer + 1) * ( margin + (slice height * unit scale)) + (layer * 20)
	
	text
		y = text height
	
	path transform
		scale = (unit scale) (-1 * unitscale)
		translate = (-1 * minX) (-1 * minY)
-->
""");

pathTemplate= Template("""
		<g id="z $zLevel" transform="translate($xTransform, $yTransform)">
			<text y="15" fill="black" stroke="black"> Layer $layerNo (z= $zLevel $units)</text>
			<path transform="scale($unitScale, -$unitScale) translate($xTranslate, $yTranslate )" d="$path"/>
		</g>
""");

bottomTemplate= Template("""
<!--End of path section-->
	</g>
	<!--End Layer Data-->
	
	<!--Button to change from all to single-->
	<g font-weight="bold" font-family="Arial" font-size="15px">
		<text id="buttonSingle" fill="darkslateblue" visibility="hidden" onclick="viewSingle()">[Single View]</text>
		
	<!--Control box for single slice layout-->
		<g id="javascriptControls" visibility="hidden" fill="\#000">
			<rect width="510" height="90" stroke="gray" stroke-width="4px" fill="silver"/>	
			<g>
				<path stroke="\#000" stroke-width="3" d="M 20 20 h5 l-5 -10 l-5 10 h5 v35 h35 v-5 l10 5 l-10 5 v-5 h-35 z"/>
				<text x="25" y="20">Y</text>
				<text x="69" y="60">X</text>
				<text x="10" y="80" id="buttonAll" onclick="viewAll();" fill="darkslateblue">[Show All]</text>
			</g>
			<g transform="translate(100, 20)">
				<text id="layerMin">1</text>
				<rect id="slider" x="10" y="-12" width="150" height="14" fill="gray"/>
				<circle id="thumb" cx="17" cy="-5" r="7" fill="darkslateblue"/>
				<text x="163" id="layerMax" >5</text>
				<text y="20" x="0">Layer </text>
				<text y="20" x="45" id="layerNum">100</text>
				<text y="20" x="138" onclick="displayLayer(currentLayer-1)" fill="darkslateblue">&lt;</text>
				<text y="20" x="153" onclick="displayLayer(currentLayer+1)" fill="darkslateblue">&gt;</text>
				<text y="40" x="0">Scale</text>
				<text y="40" x="45" id="scaleNum">0.125</text>
				<text y="40" x="138" onclick="changeScale(zoomScale/1.5)" fill="darkslateblue">&lt;</text>
				<text y="40" x="153" onclick="changeScale(zoomScale*1.5)" fill="darkslateblue">&gt;</text>
				<text y="60" id="layerThickness" >Layer Thickness: </text>
			</g>
			<g transform="translate(290, 0)">
				<text y="40">X</text>
				<text y="60">Y</text>
				<text y="80">Z</text>
				<text x="20" y="20">Min</text>
				<text id="minX" x="20" y="40">0.0</text>
				<text id="minY" x="20" y="60">0.0</text>
				<text id="minZ" x="20" y="80">0.0</text>
				<text x="80" y="20">Max</text>
				<text id="maxX" x="80" y="40">0.0</text>
				<text id="maxY" x="80" y="60">0.0</text>
				<text id="maxZ" x="80" y="80">0.0</text>
				<text x="140" y="20">Dimension</text>
				<text id="dimX" x="140" y="40">0.0</text>
				<text id="dimY" x="140" y="60">0.0</text>
				<text id="dimZ" x="140" y="80">0.0</text>
			</g>
		</g>
	</g>
	
	<!--No Javascript Control box   -->
	<g id="noJavascriptControls" fill="\#000" transform="translate(20, 1400)">
		<rect width="300" height="110" stroke="gray" stroke-width="4px" fill="silver"/>
		<g transform="translate(10, 0)">
			<g transform="translate(0, 20)">
				<text x="120">Min</text>
				<text x="170">Max</text>
				<text x="220">Dimension</text>
				<path stroke="\#000" stroke-width="3" d="M 5 40 h5 l-5 -10 l-5 10 h5 v35 h35 v-5 l10 5 l-10 5 v-5 h-35 z"/>
				<text x="3" y="20" font-weight="bold">Y</text>
				<text x="60" y="80" font-weight="bold">X</text>
				<text x="120" y="80" id="layerThicknessNoJavascript">Layer Thickness: $layerHeight $units</text>
			</g>
			<g transform="translate(100, 40)">
				<text>X</text>
				<text id="minXNoJavascript" x="20">$xMin</text>
				<text id="maxXNoJavascript" x="70">$xMax</text>
				<text id="dimXNoJavascript" x="120">$xDim</text>
			</g>
			<g transform="translate(100, 60)">
				<text>Y</text>
				<text id="minYNoJavascript" x="20">$yMin</text>
				<text id="maxYNoJavascript" x="70">$yMax</text>
				<text id="dimYNoJavascript" x="120">$yDim </text>
			</g>
			<g transform="translate(100, 80)">
				<text>Z</text>
				<text id="minZNoJavascript" x="20">$zMin</text>
				<text id="maxZNoJavascript" x="70">$zMax</text>
				<text id="dimZNoJavascript" x="120">$zDim</text>
			</g>
		</g>
	</g>
	<!--End Controls-->
	
</svg>
""");