"""
    Slicing Options.
    It is a design goal to minimize the number of these options which
    must be supplied by the user. whenever possible, we want to
    compute them.
    
    Generally we will take user options from a json string.
    
    Documentation is very important for options!
    
    Since we can compute some settings once we have the part, default values are not specified here.
    Various modules can override values if they are not specified by the user.
    
    This class is designed for user input, so all values are strings
"""

import json

class SlicingOptions:
    
    def __init__(self):
        
        #
        #options we will probably always need from the user
        #
        self.nozzleDiameter = None;                 #diameter of the nozzle, mm
        self.minLayerHeight = None;                  #the minimum layer height the machine can handle
        self.filamentDiameter = None;               #diameter of input filament, in mm
        self.fillDensity = None;                    #filling density, 1 = solid
        self.tableCenterX = None;                   #center of table in x, mm
        self.tableCenterY = None;                   #center of table in y, mm
        
        #
        #options we can probably compute in the future, but need now
        #
        self.translateToPositiveCenter = None;       #boolean     
        self.layerHeight = None;                    #ideally layerheight should be variable based on geometry
        self.generateSupports = None;               #ideally we do it all the time
        self.machineMaxFeedRate = None;             # the fastest feedrate the machine can handle
        self.trackWidth = None;                     #ideally 
               
        #
        #options we can probably compute now
        #                                            DEFAULT VALUE
        self.fixShape = None;
        self.zMin= None;                            #bottom of part
        self.zMax = None;                           #top of part
        self.units = None;                          #guess based on part
        self.fillingEnabled = None;                  #true
        self.fillingWallThickness = None;           #1mm if filling density < 1.0
        self.gcodeNumberFormat = None;              #0.000
        self.gcodeVerboseComments = None;           #false
        self.gcodeUseArcs = None;                   #true
        self.gcodeFeedRate = None;                  #1000.0
        self.gcodeAxisName = None;                  #E
        self.gcodeCurveApproxTolerance = None;      #0.001
        self.gcodeFileExtension = None;             #.gcode
        self.svgFileExtension = None;               #.svg
        self.svgNumberFormat = None;                #0.001
        
        #type conversion dictionary. Also useful later for building the GUI
        self.dTypes = {};
        self.dTypes['bool'] = "generateSupports,fillingEnable,gcodeVerboseComments,gcodeUseArcs,translateToPositiveCenter,fixShape".split(",");
        self.dTypes['float'] ="tableCenterX,tableCenterY,nozzleDiameter,minLayerHeight,filamentDiameter,fillDensity,layerHeight,machineMaxFeedRate,trackWidth,zMin,zMax,fillingWallThickness,gcodeCurveApproxTolerance".split(",") 
    
        #required attributes list. Also useful later for building the GUI
        self.requiredFields = "nozzleDiameter,minLayerHeight,filamentDiameter,fillDensity".split(",");
    
    "Returns a List of problems. Empty list means valid"
    def errors(self):
        errors = [];
        for p in self.requiredFields:
            if self.__dict__[p] == None:
                errors.append( "Required Option %s Not Set" % p );

        #if translateToPositiveCenter is specified, then table dimensions are required also
        if self.translateToPositiveCenter:
            if self.tableCenterX == None or self.tableCenterY == None:
                errors.append("translateToPositiveCenter requires tableCenterX and tableCenterY also" );

        return errors;
        
    def __str__(self):
        return self.toJson();
    
    def __repr__(self):
        return self.toJson();
    
    def toJson(self):
        return json.dumps(self.__dict__, sort_keys=True, indent=4);
    
    def loadJson(self,sJson):
        #load user options. only overwrite ones that are present, leave rest alone. conver types
        d = json.loads(sJson);
        myDict = self.__dict__;
        for[ k,v] in d.iteritems():
            #print k,v
            #if k in self.dTypes['bool']:
            #    print "found",k
            #    myDict[k] = ( v.upper() == 'TRUE');
            if k in self.dTypes['float']:
                myDict[k] = float(v);
            else:
                myDict[k] = v;


if __name__=='__main__':
    
    
    s = SlicingOptions();
    print s;
    assert s.fillingEnable == None;
    
    #set a couple of values
    s.loadJson( '{ "fillingEnable": "true" }');
    
    assert s.fillingEnable == True;
    assert s.gcodeAxisName == None;
    print s;