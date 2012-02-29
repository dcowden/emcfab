"""
    Represents calculations for the extruder
"""

class Extruder:
    def __init__(self,options):
        self.options = options;
    
    """
        compute layer height, based on options and other stuff
        TODO: shouldnt we make this more fancy?
    """    
    def layerHeight(self):
        
        #if user has selected a layerHeight, use that, otherwise for now use the minimum layer height.
        if self.options.layerHeight != None:
            return self.options.layerHeight;
        
        return self.options.minLayerHeight;
    
    """
        compute track width, based on options and other stuff
    """
    def trackWidth(self):
        #for now just use a multiple of the nozzle diameter.
        #slic3r always returns track width between 1.05 and 1.4 times nozzle diameter
        return self.options.nozzleDiameter * ( 1.05 );
    
        
    
    
    