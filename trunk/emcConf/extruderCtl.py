#!/usr/bin/python
#  Python Library for controlling the extruder via a serial communication port
#  Requires pyserial for proper operation.
#
# Designed for use either as a library or as a hal user component
#  8/2/2008
#

import serial
import string
import time
import hal
import logging
import sys
from logging import *;

HAL_POLL_INTERVAL = 3; #seconds between loops

#set up logging
logging.basicConfig(level=logging.WARN,
                    format='%(asctime)s %(levelname)s %(message)s',
                    stream=sys.stdout);
                        
print "Extruder Driver, v1.0"
def runHal():
    #set up hal component
    extruder_hal = hal.component("extruder");
    extruder_hal.newpin("temp_cmd",hal.HAL_FLOAT,hal.HAL_IN);
    extruder_hal.newpin("temp_current",hal.HAL_FLOAT,hal.HAL_OUT);
    extruder_hal.newpin("heater_duty_out",hal.HAL_FLOAT,hal.HAL_OUT);
    extruder_hal.newpin("motor_duty_out",hal.HAL_FLOAT,hal.HAL_OUT);
    extruder_hal.newpin("heater_ok",hal.HAL_BIT,hal.HAL_OUT);
    extruder_hal.newpin("heater_enable",hal.HAL_BIT,hal.HAL_IN);
    extruder_hal.newpin("motor_enable",hal.HAL_BIT,hal.HAL_IN);
    extruder_hal.ready();
    #extruder_hal.newpin("fault",hal.HAL_BIT,hal.HAL_OUT);

    #default values of gains stored in firmware are probably fine,
    #and can be set manually if required

    #ex.newpin("igain",hal.HAL_FLOAT,hal.HAL_IN);    
    #ex.newpin("pgain",hal.HAL_FLOAT,hal.HAL_IN);    
    #ex.newpin("dgain",hal.HAL_FLOAT,hal.HAL_IN);    
    #ex.newpin("ff0gain",hal.HAL_FLOAT,hal.HAL_IN);    
    #ex.newpin("ff1gain",hal.HAL_FLOAT,hal.HAL_IN); 

    #connect serial port
    extruder = Extruder();
 
    try:
            while 1:
                #copy extruder status via serial interface
                #extruder.
                newVals = {};
                extruder.open();
                newVals["ht"] = extruder_hal["temp_cmd"];
                newVals["hge"] = float(extruder_hal["heater_enable"]);
                newVals["mge"] = float(extruder_hal["motor_enable"]);
                extruder.save(newVals);
                r = extruder.read();

                extruder_hal["temp_current"] = r["hv"];
                extruder_hal["heater_duty_out"] = r["hout"];                
                extruder_hal["motor_duty_out"] = r["mout"];                
                if r["he"] > 0:
                  extruder_hal["heater_ok"] = True;
                else:
                  extruder_hal["heater_ok"] = False;
                extruder.close();    
                time.sleep(HAL_POLL_INTERVAL);
    except Exception:
            exception("Uh, i got an exception");
            raise SystemExit;

    
"""
    Class that represents an extruder,
    connected via a serial port.
    The class allows monitoring and controling the connected
    extruder
    
    Typical usage:
         ex = Extruder(baudrate=57600,comport='/dev/ttys0')
        ex.open();
        ex.setTemp(250);
           .....
        ex.close();
"""
class Extruder:
    PROMPT = "Cmd:>";
    
    def __init__(self,baudrate=57600,comport='/dev/ttyS0'):
        self._baudrate = baudrate;
        self._comport = comport;
        self.status = {};

    def open(self):
        self.connection =  serial.Serial(baudrate=self._baudrate,port=self._comport);
        self.read();

    def close(self):
        self.connection.close();
    

    #
    # Wait for confirmation, which is Cmd:>
    # and return resulting buffer.
    # 
    def _waitForPrompt(self):
        return self.connection.readline();

    #
    # send a text command to the extruder
    # each command involves setup of a new serial connection
    # each command waits for PROMPT to before returning
    #
    def _sendCommand(self,cmdString):
        res = "";
        debug("Command:" + cmdString);
        self.connection.write(cmdString + "\n");
        res = self._waitForPrompt();
        
        #the string might begin or end with the prompt,
        #so trim it out if it is there
        return string.replace(res,self.PROMPT,"");

    #
    # Command Library for the extruder
    # each command has custom arguments
    # the commands match those available in the extruder firmware.
    #
    def setTemp(self,newTemp):
		  self.open();    
        self._sendCommand("ht=" + str(newTemp) );
        self.close();
        
        
    """
    	Read extruder temp    
    """    			
    def readTemp(self):
        self.open();
        try:
        		temp = float(self.status["ht"]);
        except:
        		temp = -1;
        		
        self.close();
        return temp;   	
    """
       Updates status dictionary with the current status.         
    """
    def read(self):
        res = self._sendCommand("s");
        debug("Got Status String='" + res + "'");
        lst = string.split(res,',');
        for i in lst:
                vals = string.split(i,'=');
                if len(vals) > 1:
                        self.status[vals[0]] = float(vals[1]);
        debug ("Status = " + str(self.status) );
        return self.status;
        #reset new values

    """ 
     For each value in the status buffer, copy the commands
     into the extruder.
    """
    def save(self,newVals):
        for k in newVals.keys():
           if k in ( 'mp','mi','md','mge','mcmd','my','ht','hge','hi','hd','hp','hfg0','hy','msp' ):
              if self.status.has_key(k):
	         if newVals[k] != self.status[k]:
                      self._sendCommand(k + "=" + str(newVals[k]) );
              else:
                 self._sendCommand(k + "=" + str(newVals[k]) );

        self.read();

def mainTest():
    ex = Extruder();
    ex.open();
    print ex.status;
    temp = ex.status["ht"];
    a = {};
    a["ht"] = ex.status["ht"] + 20;
    a["hge"] = 0;
    ex.save(a);
    print ex.read();
    ex.close();
    
if __name__ == '__main__':
	runHal();
