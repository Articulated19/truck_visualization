import numpy as np
from scipy import interpolate


def spline(positions, s1, n): #positions = [(x1, y1), (x2, y2), (x3, y3)]
    if len(positions) <= 3:
        print "cant spline less than 4 positions"
        return positions
    
    
    xs,ys = zip(*positions)
    xs = np.array(xs)
    ys = np.array(ys)
    
    
    tck, u = interpolate.splprep([xs, ys], s=s1, per=False)
    xi, yi = interpolate.splev(np.linspace(0, 1, n), tck)
    return zip(xi,yi)
    
    
