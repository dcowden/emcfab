import sys,time
import numpy as np
cimport numpy as np
cimport cython
DTYPE=np.int
ctypedef np.int_t DTYPE_t


def piecewise_bresenham_line(np.ndarray[DTYPE_t,ndim=2] array not None,start,end,int minSegmentLength=0):
    """
          Brensenham line algorithm, but slightly modified:
            draws on the provided numpy array
            avoids re-drawing pixels already drawn
            returns a list of line segments that were drawn, inclusive of ends
            
          minSegmentLength is the minimum length of a segment that will be generated
          Typically this is used to eliminate single pixels length segments
  """
    cdef int steep = 0
    cdef int writing=0
    lines = []
    cdef int x0,y0,x2,y2,x,y
    cdef int sx,sy,dx,dy,i,c0,c1,lastc0,lastc1,d
     
    x0 = start[0]
    y0 = start[1]
    x2 = end[0]
    y2 = end[1]
    x = x0
    y = y0
    cdef int segX = 0
    cdef int segY = 0
     
    dx = (int)(abs(x2 - x))
     
    if (x2 - x) > 0: sx = 1
    else: sx = -1

    dy = abs(y2 - y)

    if (y2 - y) > 0: sy = 1
    else: sy = -1

    if dy > dx:
        steep = 1
        x,y = y,x
        dx,dy = dy,dx
        sx,sy = sy,sx

    d = (2 * dy) - dx
     
    #main loop more complex than normal since we check the underlying array
    for i in range(0,dx):

          if steep:
               c0=y
               c1=x
          else:
               c0 = x 
               c1=y; 
               
          if array[c0,c1] == 0:
               if writing:
                    #end segment if not too short
                    if minSegmentLength > 1:
                         length = max( abs(segX - c0), abs(segY - c1 ) )
                         if length >= minSegmentLength:
                              lines.append( [ (segX, segY),(lastc0,lastc1) ] )
                    else:
                         lines.append( [ (segX, segY),(lastc0,lastc1) ] )
                    writing = 0;
          else:
               #array[c0,c1] = 1;
               if not writing:
                    #start segment
                    writing = 1;
                    segX = c0;
                    segY  = c1;

          lastc0 = c0;
          lastc1 = c1;               
          while d >= 0:
               y = y + sy
               d = d - (2 * dx)
          x = x + sx
          d = d + (2 * dy)
          
    #finish up
    if writing:
    #end segment
        #array[x2,y2] = 1;
        lines.append( [(segX,segY),(x2,y2)] );
    
    return lines;
     

def testPieceWiseBresenhamPerformance():
     "test performance"
     q = time.clock();
     cdef np.ndarray[DTYPE_t,ndim=2] a
     a  = np.zeros((1000,1000),dtype=np.int );
     LOOPS=200;
     for i in range(0,LOOPS):
          lines  = piecewise_bresenham_line(a,(i,0),(i+i,i) );
     print "%d iters, %0.5f sec per iter" %  (LOOPS, (time.clock() - q )/LOOPS);
     
def testPieceWiseBresenham():
     """
          Tests code that does piecewise bresenham algorithm.
          
     """
     
     # a stripe down the middle
     cdef np.ndarray[DTYPE_t,ndim=2] a
     a  = np.zeros((12,12),dtype=np.int );
     a[:,5:7] =1;
     #print a;

     #draw a vertical line
     P1,P2 = (0,9),(0,10);
     lines = piecewise_bresenham_line(a,P1,P2);
     assert len(lines) == 1, "One Line"
     assert lines[0] == [ P1,P2 ], "Should be simple line"

     #x=y line.
     P1,P2=(0,0),(9,9);
     lines = piecewise_bresenham_line(a,P1,P2 );
     assert len(lines) == 2, "Should be two segments"
     assert lines[0][0] == P1, "First point should be (0,0)"
     assert lines[0][1] == ( 4,4 ), "Second point in first line should be (4,4)"
     
     #horizontal line. corner overlaps with previous
     lines = piecewise_bresenham_line(a,( 0,0 ),(5,0 ) );
     assert lines[0] == [ (1,0),(5,0) ], "First Line should be (1,0) --> (5,0). 0,0 was already covered "
     assert len(lines) == 1, "Should be one line segment"
     assert lines[0][0][0] == 1,"First Point should start at x=1"

     #vertical line that creates single unfilled pixel
     lines =  piecewise_bresenham_line(a,( 2,0 ),(2,6 ),2 );
     assert lines[0] == [ (2,3 ), (2,4 )], "Line should skip (2,0) is already covered, (2,1) \
         is open but short segment, (2,2) is covered, line goes 2,3 - 2,4"
     assert len(lines) == 1, "Should be one line"

     #line that produces no drawing
     P1,P2 = (2,2),(4,4);
     lines = piecewise_bresenham_line(a,P1,P2);
     assert len(lines) == 0, "Should be no entries in this line"
               

if __name__=='__main__':
     print "Running Tests....."
     testPieceWiseBresenham();
     print "Testing Performance..."
     testPieceWiseBresenhamPerformance();
     
     