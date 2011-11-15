sCommands = {	 
	0: 0 ,
	0.1: 2,
	0.2: 4,
	0.3: 6,
	0.4: 8,
	0.5: 10,
	5.0:8,
	5.1:6,
	5.2:4,
	5.3:2,
	5.4:1,
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
			
print findCommand(0.001);
print findCommand(0);
print findCommand(0.151);
print findCommand(6 );
print findCommand(5.201);