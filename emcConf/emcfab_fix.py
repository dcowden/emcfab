"""
	Fixes Skeinforge output to 
	remove crap that emc does not need.
	http://reprap.org/wiki/MCodeReference
	we need to change: 
		M104: set temp ( but needs to be M104 Pxxx.yy not S )
	
	we need to remove:
		M101 extruder on
		M105 extruder get temp
		M113 extruder pwm set
		M108 extruder speed
		M103 extruder stop
"""
import sys
import os
import os.path

if len(sys.argv) < 2:
	print "Usage: python emcfab_fix.py <file_to_read>"
	sys.exit(1);
	
loc = os.path.join(os.getcwd(),sys.argv[1]);
if not os.path.isfile(loc):
	print "Cannot find file'",loc,"'"
	sys.exit(1);
	


#compute new names
(path,file) = os.path.split(loc)
t = file.split(".")

newfile = os.path.join(path,t[0] + "_fixed.nc" );
print "Fixing Up  File'",loc,"'"
print "Creating output file'",newfile,"'"

inF = open(loc)
outF = open(newfile,'w')

for line in inF:
	if line.startswith("M101") or line.startswith("M105") or \
	   line.startswith("M113") or line.startswith("M108") or \
	   line.startswith("M103"):
		continue;
	elif line.startswith("M104"):
		#adjust format
		outF.write(line.replace('S','P')  );
	else:
		outF.write(line );

outF.write("M02\n");
print "Done."
inF.close();
outF.close();