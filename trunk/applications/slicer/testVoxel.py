from OCC.Display.SimpleGui import *
from OCC import BRepPrimAPI
from OCC import Voxel


display, start_display, add_menu, add_function_to_menu = init_display()
box = BRepPrimAPI.BRepPrimAPI_MakeBox(10,10,20)
box_voxel = Voxel.Voxel_ColorDS()
v = Voxel.Voxel_FastConverter(box.Shape(),box_voxel,0.1,20,20,20)
progress=1
v.Convert(progress)
v.FillInVolume(1);
visu = Voxel.Voxel_Prs()
visu.SetBoolVoxels(box_voxel)
visu.SetDisplayMode(0);
visu.SetColor(2);
visu.SetPointSize(1);
visu.SetQuadrangleSize(100);
visu.SetTransparency(0);
#display.DisplayShape(box.Shape())
display.Context.Display(visu.GetHandle(),True)
start_display()