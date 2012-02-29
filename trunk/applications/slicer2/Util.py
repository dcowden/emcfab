"""
    Utility functions of general use
"""

     
class Timer:
    def __init__(self):
        self.startTime = time.time();
        self.startAscTime = time.asctime();
        
    def start(self):
        return self.startTime;
            
    def elapsed(self):
        end = time.time();
        return end - self.startTime;
        
    def finishedString(self):
        return "%0.3f sec" % ( self.elapsed() );

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return itertools.izip(a, b)
    
def ntuples(lst, n,wrap=True):
    B = 0;
    if not wrap:
        B = 1;
    return zip(*[lst[i:]+lst[:(i-B)] for i in range(n)])
    
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