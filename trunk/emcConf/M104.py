#!/usr/bin/python
#
# M-app for extruder temperature control.
# Sets extruder temp, and waits until temp is at setpoint
# For stability, open and close the connection
# instead of holding it open
#
# Arg1 is the temp setpoint in C, arg2 is tolerance ( in degC)

import extruderCtl
import sys,time,math


POLL_INTERVAL = 5; #in seconds
TIMEOUT = 1000; #in seconds

 
if __name__ == '__main__':

	#save new setpoint
	newTemp = float(sys.argv[1]);
	tolerance = float(sys.argv[2]);
	ex = Extruder();
		
	ex.setTemp(newTemp);
	
	#wait till we reach the setpoint, or timeout
	t = 0;
	while t < TIMEOUT:
		x = ex.readTemp();
		if ( math.fabs(x - newTemp) <= tolerance):
			break;
		else:
			t += POLL_INTERVAL;
			time.sleep(POLL_INTERVAL );

		