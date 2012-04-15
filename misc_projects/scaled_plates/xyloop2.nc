( Scaled License Plate for Craig Varney )
( scaled_plate 4/14/2012 8:48:29 AM )
( uses 92 offsets because fixture offsets limit to 6 copies)
( can also be considered a template for making any array of similar items )
( T0 : 0.125 )

( === VARIABLES === )
#1 = 0.125 ( clearance plane ) 
#2 = 1.5 ( x spacing )
#3 = 1.0 ( y spacing )
#4 = 5.0 ( maximum in x direction )
#5 = 5.0 ( maxiumum in y direction )
#6 = 0.0 ( current x grid location )
#7 = 0.0 ( current y grid location )
#8 = 2.5 ( end z move  )

( == MAIN PROGRAM-- repeats program below for a grid of sections == )
G20 G90 G64 G40 ( inches, absolute mode, path optimized mode, no cutter comp )
T0 M6
M3 S16000
G92.1 ( cancel any offsets )
G0 X0 Y0 Z#1 ( go to origin, clearance plane )

O101 while [ #6 lt #4 ] ( x loop )
	
	O110 while [#7 lt #5 ] ( yloop)
		O103 call
		#7 = [#7 + #3 ]
		O200 call #6 #7 0.125 ( move to new location in global coordinates )
		G92 X0.0 Y0.0
	O110 endwhile	
	#6 = [ #6 + #2 ]
O101 endwhile

G92.1( clear offsets and set to zero )
G00 X0 Y0 Z#8

M5
M30


( === PUT PROGRAM YOU WOULD LIKE TO REPEAT HERE === )
O103 sub
	G0 X0.7724 Y0.2262
	G1 F10.0 Z0.0
	G3 F80.0 X0.7438 Y0.0594 Z-0.03 I0.4713 J-0.1669
	G1 Y0.3624
	G3 X0.6613 Y0.4449 I-0.0825 J0.0
	G1 X0.0493
	G3 X-0.0332 Y0.3624 I0.0 J-0.0825
	G1 Y0.0594
	G3 X0.0493 Y-0.0231 I0.0825 J0.0
	G1 X0.6613
	G3 X0.7438 Y0.0594 I0.0 J0.0825
	G0 Z0.125
O103 endsub
( === END MAIN PROGRAM === )

(=== Global Move in unadjusted coordinates === )
O200 sub
    G92.2
	G00 X#1 Y#1 Z#3
	G92.3
O200 endsub