( Machine a semicircular slot to hold diamter 0.072 tube)
( T0 : 0.0625 ball end mill )
( y=0 at center of slot )
( z=0 at stock surface )
( x=0 at left of slot )
G20 G90 G64 G40
G0 Z0.25
G17
M3 S2000
( Machine to depth of ball end )
#1=0.0 ( left end  x)
#2=4.0 ( right end x)
#3=80.0
#4=0.1 (clear plane)
 
g00 x#1 y0.0 z#4

(first, cut to depth with ball end mill )
G01 z-0.005 f20.
g01 x#2 f#3
g01 z-0.010 
g01 x#1
g01 z-0.015
g01 x#2
g01 z-0.020
g01 x#1
g01 z-0.025
g01 x#2
g01 z-0.030
g01 x#1
g01 z-0.036
g01 x#2
g00 z#4
g00 x#1 y0.0

(now we have machined the center trough, carve out the edges )
(we start at the center and work outwards )
g00 y0.001
g01 z-0.0359 f20.
g01 x#2 f#3
g01 y0.002 z-0.0356
g01 x#1
g01 y0.003 z-0.0349
g01 x#2
g01 y0.004 z-0.0338
g01 x#1
g01 y0.0045 z-0.0328
g01 x#2
g01 y0.0048 z-0.0313
g01 x#1
g00 z#4
g00 x#1 y0.0

(same as above but negative y instead )
g00 y-0.001
g01 z-0.0359 f20.
g01 x#2 f#3
g01 y-0.002 z-0.0356
g01 x#1
g01 y-0.003 z-0.0349
g01 x#2
g01 y-0.004 z-0.0338
g01 x#1
g01 y-0.0045 z-0.0328
g01 x#2
g01 y-0.0048 z-0.0313
g01 x#1
g00 z#4
g00 x#1 y0.0


(program complete)
M5
M30