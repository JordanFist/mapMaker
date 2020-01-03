from math import atan2, sqrt
from bresenham import bresenham

def pathToObstacle(robotSquare, obstacleSquare):
    return list(bresenham(robotSquare[0], robotSquare[1], obstacleSquare[0], obstacleSquare[1]))

def normalize(V):
    norm = sqrt(V['X']**2 + V['Y']**2)
    V['X'] /= norm
    V['Y'] /= norm

def determinant(U, V):
    return U['X'] * V['Y'] - U['Y'] * V['X'] 
    
def scalarProduct(U, V):
    return U['X'] * V['X'] + U['Y'] * V['Y']

def getAlpha(A, B, U):
    V = {'X':B['X'] - A['X'], 'Y':B['Y'] - A['Y']}
    normalize(V)
    normalize(U)
    det = min(1, max(-1, determinant(U, V)))
    scal = min(1, max(-1, scalarProduct(U, V)))
    return 2 * atan2(det, scal)
    
def getRadius(A, B, U):
    t = ((B['X'] - A['X'])**2 + (B['Y'] - A['Y'])**2) / (2 * (U['Y'] * (B ['X'] - A['X']) - U['X'] * (B ['Y'] - A['Y'])))
    xc = A['X'] + t * U['Y']
    yc = A['Y'] - t * U['X']
    return sqrt((A['X'] - xc)**2 + (A['Y'] - yc)**2)
   
def getDistance(A, B):
    return sqrt((A['X'] - B['X'])**2 + (A['Y'] - B['Y'])**2)

def getLineCircleIntersection(C, radius, A, B):
    """
    :param C: center of the circle
    :param radius: radius of the circle
    :param A: one point of the line
    :param B: another point of the line
    :return: the intersections (we assume delta > 0)
    """
    if B['X'] == A['X']:
        x1 = x2 = B['X']
        y1 = sqrt(radius * radius - (x1 - C['X']) ** 2) + C['Y']
        y2 = -sqrt(radius * radius - (x2 - C['X']) ** 2) + C['Y']
    else:
        m = (B['Y'] - A['Y']) / (B['X'] - A['X'])
        a = 1 + m * m
        b = -(2 * m * m * A['X'] + 2 * C['X'] + 2 * m * (C['Y'] - A['Y']))
        c = -radius * radius + C['X'] * C['X'] + (A['Y'] - C['Y']) ** 2 + A['X'] * A['X'] * m * m - 2 * m * A['X'] * (
                    A['Y'] - C['Y'])
        delta = b * b - 4 * a * c
        x1 = (-b + sqrt(delta)) / (2 * a)
        x2 = (-b - sqrt(delta)) / (2 * a)
        y1 = m * (x1 - A['X']) + A['Y']
        y2 = m * (x2 - A['X']) + A['Y']
    return {'X': x1, 'Y': y1}, {'X': x2, 'Y': y2}



