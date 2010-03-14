"""
	Utilities to fill a slice using the 'pacman' algorithm
	The algorithm works by following these basic steps
	
	(1) Analysis phase
		(a) first, lay a fixed space grid over the entire area
		(b) map the boundaries onto the grid
		(c) mark those grid points that are on and not in the face
	(2) Fixup phase
		(a) find all grid sections that are only one pass wide, and create paths for them
		(b) find all grid sections that can only fit MIN_SIZE passes and create paths for those too
	
	(3) Outline  phase
		(a) Inset the boundary by 1/2 road width and create boundary toopaths
		(b) Inset this boundary NUM_SHELLS times inside ( if possible ) and create these paths
	
	(4) Filling phase
		(a) starting with an interior section, start cross hatching, following the hatch direction
			
"""


"""
     a point in a grid
	 Each point in a grid stores two pieces of information:
	    whether it is on the surface of the solid
		whteher it has been covered by a line of filament yet
"""
class gridpoint:

	def __init__(self,x,y):
		self.x = x;
		self.y = y;
		self.onsurface = False;
		self.filled = False;
		
	def needsFill(self):
		return self.onsurface and (not self.filled );
		
	
"""
	a grid of points, each of which can store state
"""
class booleangrid:

	def __init__(self,spacing):
		

		

		