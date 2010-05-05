from  SlicerConfig import *


"""
	Test Settings
	For Quicker Slicing
	
"""
userOptions = SlicerOptions();
#userOptions.filling.enabled=False;
userOptions.filling.fillWidth=0.3;
userOptions.filling.numExtraShells=50;
userOptions.useSliceFactoring = True;
userOptions.filling.checkFillInterference=False;
userOptions.layerHeight=1.0;
userOptions.gcode.computeExtraAxis = False;
