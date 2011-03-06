import Wrappers
import timeit
from OCC import gp

e = Wrappers.edgeFromTwoPoints(gp.gp_Pnt(0,0,0),gp.gp_Pnt(1,1,1));
e2
t = Wrappers.Timer();

#conclusion of this test:
#__hash__ is slightly faster.
#__hash__ is much more readable, and works with built-in python

N = 1000000;
for i in range(1,N):
	e.__hash__();
print t.finishedString();

t = Wrappers.Timer();
for i in range(1,N):
	e.HashCode(1000000000);
print t.finishedString();
