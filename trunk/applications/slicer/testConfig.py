from  SlicerConfig import *


"""
	Test Settings
	For Quicker Slicing
	
"""
userOptions = SlicerOptions();
#userOptions.filling.enabled=False;
userOptions.filling.fillWidth=0.3;
userOptions.filling.numExtraShells=4;
userOptions.layerHeight=2.0;
userOptions.gcode.computeExtraAxis = False;
