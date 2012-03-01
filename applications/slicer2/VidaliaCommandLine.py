"""
    Command Line interface for the slicer.
    
"""
import os,os.path,sys

import Slicer
import SlicingOptions
import SolidFileReader
import Exporters

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
    #slicer.display(); #show the result
    
    #compute output files based on input file location
    (path,file) = os.path.split(inputFile);
    rootFileName = file.split(".")[0]
    
    outputGcodeFile = os.path.join(path,rootFileName + ".gcode");
    outputSVGFile = os.path.join(path,rootFileName + ".svg");
    
    print "GCODE FILE: %s" % outputGcodeFile
    print "SVG FILE: %s" % outputSVGFile

    #gce = Exporters.GcodeExporter(opts);
    #sve = Exporters.SvgExporter(opts);
    
    #gf = open(outputGcodeFile,'w');
    #sf = open(outputSVGFile,'w');
    
    #gce.export(slicer,gf);
    #sve.export(slicer,sf);
    
    #gf.close();
    #sf.close();
    
    