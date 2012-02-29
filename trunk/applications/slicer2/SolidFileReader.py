"""
    Reads an input file, and renders a shape as a result
""" 

import sys,os,os.path

from OCC import StlAPI,STEPControl,TopoDS


def determineFileType(fileName):
    """
       Open the file to determine what kind of file it is.
       returns STEP if the file is a step file, STL if it is STL, ??? otherwise
    """

    if fileName.upper().endswith("STL"):
        return "STL";
    
    if fileName.upper().endswith("STEP"):
        return "STEP";
    
    #read the first 4 lines of the file, since we couldnt figure it out by file extension
    f = open(file,'r');
    text = "";                
    for i in range(5):
        text = text + f.readline();
    
    f.close();
    if 'FILE_DESCRIPTION' in text or 'STEP' in text:
         return "STEP";
    elif 'solid' in text: 
         return "STL";
    else:
        return "???";

"""
    read a file, return a solid object
"""
class SolidFileReader:
    def __init__(self):
        pass;
    
    def readFile(self,inputFileName):
        if not os.path.exists(inputFileName):
            raise ValueError, "Error: '%s' Does not Exist!" % inputFileName;        
        t = determineFileType(inputFileName);
        
        if t == "STEP":
            return self._readSTEP(inputFileName);
        elif t == "STL":
            return self._readSTL(inputFileName);
        else:
            raise ValueError, "Cannot Read '%s': Unknown file type!";
        
    def _readSTL(self,inputFileName): 
        ts = TopoDS.TopoDS();
        shape = TopoDS.TopoDS_Shape()
        stl_reader = StlAPI.StlAPI_Reader()
        stl_reader.Read(shape,inputFileName)
        return shape;


    def _readSTEP(self,inputFileName):
        stepReader = STEPControl.STEPControl_Reader();
        stepReader.ReadFile(inputFileName);
        numItems = stepReader.NbRootsForTransfer();
        numTranslated = stepReader.TransferRoots();
        return stepReader.OneShape();

if __name__=='__main__':
    
    from OCC.Display.SimpleGui import *
    display, start_display, add_menu, add_function_to_menu = init_display()
    
    file1 = "c:/data/emcfab/applications/slicer/parts/RepRaptest.STEP";    
    file2 = "c:/data/emcfab/applications/slicer/parts/RepRaptest.stl";
    
    sr = SolidFileReader();
    
    o1 = sr.readFile(file1);
    o2 = sr.readFile(file2);
    
    display.DisplayShape(o1);
    display.DisplayShape(o2);
    start_display();   