#
# simulates melt flow compensation
# to make sure i have my equations right
import math

"""
	A floating point range generator
"""		
def frange6(*args):
    """A float range generator."""
    start = 0.0
    step = 1.0

    l = len(args)
    if l == 1:
        end = args[0]
    elif l == 2:
        start, end = args
    elif l == 3:
        start, end, step = args
        if step == 0.0:
            raise ValueError, "step must not be zero"
    else:
        raise TypeError, "frange expects 1-3 arguments, got %d" % l

    v = start
    while True:
        if (step > 0 and v >= end) or (step < 0 and v <= end):
            raise StopIteration
        yield v
        v += step	




#step commands-- defines the commanded profile.
#first value is the global time, second is the value of steps during that time interval
#and for all those afterwards until the next value
sCommands = {	 
	0: 0 ,
	0.002: 20,
	0.1: 40,
	0.2: 60,
	0.3: 100,
	0.4: 200,
	2.5: 200,
	2.6: 0
#	5.0:8,
#	5.1:6,
#	5.2:4,
#	5.3:2,
#	5.4:0,
};

def findCommand( value):
	"find the current step in the profile";
	kk = sCommands.keys();
	kk.sort();
	
	v = kk[0];
	for k in kk:
		if k > value:
			return sCommands[v];
		else:
			v = k;
	return sCommands[v];


#constants
Tmf = 1.7					#melt flow time constant
#Tex = 0.08					#extruder time constant
mf = 0.1					#melt flow adjust, for ABS
pmf = ( 1 + mf )			#melt flow adjust, for ABS
dt = 0.01 					#the servo loop interval, in seconds
in_step = 1.18 / 80000 / math.pow(0.012,2) * math.pow(0.03125,2);				#inches per commanded step-- one step per thousanth.

simMin = 0.0				#print simulation starting with t=0
simMax = 6.0				#print simulation till t=6 
dtSim = 0.2					#time points every 0.2 seconds	
	
#initialize variables
#Qis = 0;
#Qmf = 0;
#dQmf = 0;
#Qtarget = 0;
vTarget = 0;
xTarget = 0;
xAdjusted = 0;
stepCommand = 0;

xMelt = 0;
dxMelt = 0;
leftOver = 0; #fractional step adjust remaining from previous step(s)

outData = [];  #t, vTarget,vAdjusted, xTarget,xAdjusted,xMelt,dxMelt
outData.append ( "\t".join( [ "t", "vTarget","vAdjusted","xTarget","xAdjusted","xMelt","dxMelt","stepCommand","stepAdjust"] ) );
commandIndex = 0;

#compute the simulation
for t in frange6(simMin,simMax,dt ):

	#find number of steps for this interval
	stepCommand = findCommand(t);
	
	#commanded positions and velocities
	#method1: long way
	
	#mul:3  add/sub:6   div:2 round:1
	dx = pmf * ( stepCommand - xMelt );
	stepAdjust = dx - stepCommand + leftOver; #this could be a floating point number
	tmp = (int)(stepAdjust);
	leftOver = stepAdjust - tmp;
	dxMelt = (mf/pmf*dx - xMelt ) * dt / Tmf; #note that dt and Tmf are constants and can be combined, as well as mf/pmf
	xMelt = xMelt + dxMelt;

	#optimized version:
	#mul:3   add/sub:4  div:0 round:1
	A = mf / pmf * dt / Tmf;  #can be a program constant
	C = 1 - (dt / Tmf);  #another program constant
	
	dx = pmf * ( stepCommand - xMelt );
	stepAdjust = dx - stepCommand + leftOver;
	tmp = (int)(stepAdjust);
	leftOver = stepAdjust - tmp;
	xMelt = A*dx + ( C*xMelt);
	
	

	#these are needed only for diagnostics, not for closed loop compensation
	vAdjusted = dx/ dt * in_step; 
	xAdjusted = dx * in_step;
	xInc = stepCommand * in_step;
	vTarget = xInc / dt;
	xTarget += xInc;
	
	#store the structure for printout
	#outData.append ( "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t" % ( t, vTarget,xTarget,vMelt,dvMelt,vAdjusted,xAdjusted ) );
	outData.append ( "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f" % ( t, vTarget,vAdjusted, xTarget,xAdjusted,xMelt,dxMelt,stepCommand,tmp ) );
	
#simulation complete-- write data to output file
f = open('simulation.csv','w');
for d in outData:
	f.write(d + "\n" );
f.close();
	
	
	