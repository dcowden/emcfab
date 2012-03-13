from OCC.Display.SimpleGui import *
display, start_display, add_menu, add_function_to_menu = init_display()

"""
    jeeze is there a way to automate the creation of a delegate in python? you'd think...
"""

def DisplayShape(shapes, material=None, texture=None, update=False):
    display.DisplayShape(shapes, material, texture, update)

def DisplayColoredShape(shapes, color, update=False):
    display.DisplayColoredShape(shapes, color, update);

def EraseAll():
    display.EraseAll();
    
def FitAll():
    display.FitAll();
    
def start():
    start_display();