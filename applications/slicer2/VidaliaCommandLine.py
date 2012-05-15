"""
    Command Line interface for the slicer.
    
"""
import os,os.path,sys

import Slicer
import SlicingOptions
import SolidFileReader
import SvgExporter,GcodeExporter,ImageExporter

def printUsage():
    print """
Usage: VidaliaCommandLine <inputfile: STL or STEP file> <configurationfile>
    - inputfile is an STL or STEP file
    - configurationfile is a configuation file containing configuration information, see example.json
        """

if __name__=='__main__':
     
    nargs = len(sys.argv);
    
    if nargs < 3:
        printUsage();
        sys.exit(1);
    
    #input files
    inputFile = sys.argv[1];
    configFile = sys.argv[2];
    

    s = SolidFileReader.SolidFileReader();    
    shape = s.readFile(inputFile);

    opts = SlicingOptions.SlicingOptions();    
    txt = open(configFile,'r').read();    
    opts.loadJson(txt);

    slicer = Slicer.Slicer(shape,opts);
    
    # 
    #TODO: need to be able to pass in progress monitor here for occasional chances to cancel and to report progress
    #
    slicer.execute(); #run the slice operation. this runs a potentially long time.
    
    
    #compute output files based on input file location
    (path,file) = os.path.split(inputFile);
    rootFileName = file.split(".")[0]
    

    outputSVGFile = os.path.join(path,rootFileName + ".svg");
    print "SVG FILE: %s" % outputSVGFile
    sve = SvgExporter.SvgExporter(slicer,opts);
    sve.export(outputSVGFile);

    #outputGcodeFile = os.path.join(path,rootFileName + ".gcode");    
    #print "GCODE FILE: %s" % outputGcodeFile
    #gce = GcodeExporter.GcodeExporter(slicer,opts);           
    #gce.export(outputGcodeFile);

    outputImageFile = os.path.join(path,rootFileName + ".zip");
    print "SVG FILE: %s" % outputImageFile
    sve = ImageExporter.ImageExporter(slicer,opts);
    sve.export(outputImageFile);

    #slicer.display(); #show the result
    
    