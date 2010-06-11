"""
	Configuration Classes
	This product is highly configurable, and 
	as such configuration is an important part of the 
	product.
	
	There are validation and configuration frameworks out there,
	but I need to keep dependencies down, and stay flexible.
	
"""

UNITS_MM = "mm";
UNITS_IN = "in";
UNITS_UNKNOWN = "units";


class BaseOption:
	
	"Base Option Class, provides some utilities"
	def merge(self,otherOptions):
		"transfer properties from the supplied object to this one, but only if provided"
		myD = self.__dict__;
		oD = otherOptions.__dict__;
		for[ k,v] in oD.iteritems():
			#recurse into child objects
			if isinstance(v,BaseOption):
				myD[k].merge(v);
			else:
				if (oD[k] != None) and myD.has_key(k) :
					myD[k] = oD[k];

		
	def __str__(self):
		"string representation"
		s = [];
		for k in self.__dict__.keys():
			s.append(('%s: %s') % ( k, self.__dict__[k] ));
		return "\n".join(s);
"""
	Options for Slicing 
"""
class SlicerOptions(BaseOption):
	"options related to slicing"
	
	def __init__(self):
	
		self.units = None;
		self.nozzleDiameter = None;
		self.zMin = None;
		self.zMax = None;
		self.layerHeight = None;
		self.translateToPositiveSpace = None;
		self.DEFAULT_RESOLUTION = { UNITS_MM : 0.3, UNITS_IN : 0.012 };
		
		self.useSliceFactoring = True;
		
		#other groups of options
		self.filling = FillingOptions();
		self.gcode = GcodeOptions();
		self.svg = SVGOptions();
	
	def setDefaults(self,uom):
		"set sane defaults based on uom"
		self.filling.setDefaults();
		self.gcode.setDefaults();
		self.svg.setDefaults();
		
		if uom == UNITS_MM or uom == UNITS_IN:
			self.units = uom;
			self.nozzleDiameter = self.DEFAULT_RESOLUTION.get(uom);
			
			#default layerheight is same as diameter
			self.layerHeight = self.nozzleDiameter;
						
			#default filling options
			self.filling.fillWidth = self.nozzleDiameter;	
		else:
			raise ValueError,"Invalid Unit of Measure '%s'" % uom;



		
	"""
		self.addItem("units","string","Model Unit of Measure","AutoDetect from Model" );
		self.addItem("nozzleDiameter","float","Nozzle Diameter","0.3 mm or 0.012 In" );
		self.addItem("zMin","float","Value to Start slicing from","Minimum Model  Value" );
		self.addItem("zMax","float","Value to stop slicing","Maximum Model Value" );
		self.addItem("numSlices","int","Number of Slices to generate","Compute from layerHeight");
		self.addItem("layerHeight","float","Height of each slice layer","same as nozzle diameter" );
		self.addItem("translateToPositiveSpace","boolean","Should the object be moved to positive coordinates?","True" ,True);		
		self.addItem("enabled","boolean","Enable Filling of Slices.","True" ,True);
		self.addItem("fillWidth","float","Width of filling, in model units.","Use LayerHeight" );
		self.addItem("numExtraShells","int","Number of extra shells to compute after the first border","Generate as many shells as possible before switching to infill",99999999);
		self.addItem("checkFillInterference","boolean","Check For Fills that are too close to other fills. Use for irregular shapes. Improved performance if disabled","True" ,True);
	"""

class FillingOptions(BaseOption):
	"options related to filling"
	def __init__(self):
		self.enabled = None;
		self.fillWidth = None;
		self.numExtraShells = None;
		self.checkFillInterference = None;
		
	def setDefaults(self):
		"set default values. Not in constructor so we can merge properly "
		self.enabled = True;
		self.numExtraShells= 5;
		self.checkFillInterference = True;

class GcodeOptions(BaseOption):
	"options related to gcode creation"
	def __init__(self):
		self.enabled = None;
		self.useArcs = None;
		self.numberFormat = None;
		self.computeExtraAxis = None;
		self.extraAxisName = None;
		self.machineResolution = None;
		self.fileExtension =None;
		self.curveApproximationTolerance =None;
		self.commentFormat = None;
		self.feedRate = None;
	
	def setDefaults(self):
		self.enabled = True;
		self.useArcs = True;
		self.numberFormat = '%0.3f';
		self.computeExtraAxis = True;
		self.extraAxisName = 'A';
		self.machineResolution = 0.00005;
		self.fileExtension = '.nc';
		self.curveApproximationTolerance = 0.0001;
		self.commentFormat = "( %s )";
		self.feedRate = 100;

	
	"""
		self.addItem("enabled","boolean","Output Gcode Files.","True" ,True);
		self.addItem("useArcs","boolean","Output Gcode Arcs (G02/G03). Much more compact if your machine can handle it.","True" ,True);
		self.addItem("numberFormat","string","python string format for numbers","%0.3f" ,'%0.3f');
		self.addItem("computeExtruderAxis","boolean","Should an extra axis be created that computes the total distance of each move?","True" ,True);
		self.addItem("extruderAxisName","string","If computExtruderAxis is true, what is the name of the axis?","A" ,'A');
		self.addItem("machineResolution","float","What value is small enough that your machine considers two points closer the same value?","0.0005" ,0.0005);
		self.addItem("fileExtension","string","What file extension should be used to append to files for gcode output?",".nc" ,".nc");
		self.addItem("curveApproximationTolerance","float","If usearcs is false, what deflection is acceptable when approximating with line segments?","0.0005",0.0005);
		self.addItem("commentFormat","string","Python format string for creating comments.","'( %s )'","'( %s )'" );
		self.addItem("feedRate","float","Feedrate to use for moves.","100 Units/Min",100.0 );
	"""
	
class SVGOptions(BaseOption):
	"options related to svg creation"
	def __init__(self):
		self.enabled = None;
		self.numberFormat = None;
		self.fileExtension = None;
	
	def setDefaults(self):
		self.enabled = True;
		self.numberFormat = '%0.3f';
		self.fileExtension = '.svg';
		
	"""
		self.addItem("enabled","boolean","Output SVG Files.","True" );
		self.addItem("numberFormat","string","python string format for numbers","%0.3f" );
		self.addItem("fileExtension","string","What file extension should be used to append to files for svg output?",".nc" );	
	
	"""


if __name__=='__main__':
	"make sure these objects work correctly."
	
	options1 = SlicerOptions();
	options2 = SlicerOptions();
	options1.units = 'mm';
	options1.filling.enabled = False;
	options2.filling.enabled = True;
	options1.zMin = 2;
	options2.units = 'in';
	options1.gcode.enabled = True;

	assert options1.filling.enabled == False, "Should start out false."
	options1.merge(options2);
	
	assert options1.filling.enabled == True, "Filling should be true"
	assert options1.units =='in', "units should be in"
	assert options1.zMin == 2, "zMin should be 2"

	
	