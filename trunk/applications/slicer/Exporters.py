import os
import sys
import os.path
import logging
import time
import math

"""
	A Generator creates a file
	from a stream of path events.
	
	This allows creating adapters that create
	multiple formats
	
	The slice library will already compute the paths necessary
"""
class SVGPathGenerator:
	def __init__(self):
		#initialize stuff here
		self.pointFormat = "%0.4f %0.4f ";
		self.paths = [];
		
	def moveTo(self,x,y,z):
		self.paths.append("M " + self.pointFormat %( x, y) );
	
	def lineTo(self,x,y,z):
		self.paths.append("L " + self.pointFormat % (x, y) );
		
	def result(self):
		return self.paths.join("");

class GCodePathGenerator:
	def __init__(self,feedRate,zLevel):
		self.pointFormat = "X%0.4f Y%0.4f Z%0.4f F%0.4f ";
		self.paths = [];	
		self.feedRate = feedRate;
		
	def moveTo(self,x,y,z):
		self.paths.append("G00 " + self.pointFormat %( x, y,z,self.feedRate ));
	
	def lineTo(self,x,y,z):
		self.paths.append("G01 " + self.pointFormat %( x, y,z,self.feedRate ));
		
	def result(self):
		return self.paths.join("\n");		