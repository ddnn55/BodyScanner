
def plusEq(a,b):
    for i in range(3):
        a[i] += b[i]

def mult(vec, scalar):
    return (vec[0]*scalar,vec[1]*scalar,vec[2]*scalar)

def subtract(A,B):
    return [a-b for a,b in zip(A,B)]

def add(A,B):
    return tuple(a+b for a,b in zip(A,B))

#def addAll(*V):
#    return tuple(sum(vals) for vals in zip(*V))

def addAll(A,B,C,D):
    return tuple(a+b+c+d for a,b,c,d in zip(A,B,C,D))

def dot(A,B):
    return sum(a*b for a,b in zip(A,B))

# def absolute(frame, vector):
#     return addAll(frame.o,
#         mult(frame.x, vector[0]),
#         mult(frame.y, vector[1]),
#         mult(frame.z, vector[2])
#     )
# 
# def relative(frame, vector):
#     offset = subtract(vector, frame.o)
#     return (
#         dot(frame.x, offset),
#         dot(frame.y, offset),
#         dot(frame.z, offset)
#     )

def absolute(frame, vector):
    return add(vector, frame.o)
def relative(frame, vector):
    return subtract(vector, frame.o)


from math import sqrt
def mag((x,y,z)):
    return sqrt(x*x + y*y + z*z)

class Frame(object):
    def __init__(self, x,y,z,o):
        self.x = x
        self.y = y
        self.z = z
        self.o = o