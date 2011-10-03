import yaml


from math import sqrt
def Mag(x,y,z):
    return sqrt(x*x + y*y + z*z)

def GetPos(sk, jointName):
    # maybe the _1 indicates that it tracks several people?
    joint = sk.get('/'+jointName+'_1',None)
    if joint is None:
        return (None,None,None)
    else:
        tr = joint['translation']
    return ( # shuffle so that the skelton is upright, remember to do this transform on the mesh too
        float(tr[1].replace('y -- ', '')),
        float(tr[2].replace('z -- ', '')),
        float(tr[0].replace('x -- ', ''))
    )

def GetLen(sk, jointName, parentName):
    x,y,z = GetPos(sk, jointName)
    px,py,pz = GetPos(sk, parentName)
    if x is None or px is None:
        return None
    return Mag(x-px, y-py, z-pz)

def Lengths():
    skel = yaml.load(open('../recordings/david2/david2_skeleton.yaml','r'))
    for timestamp,sk in sorted(skel.iteritems()):
        yield [
          GetLen(sk, 'head', 'neck'),
          GetLen(sk, 'neck', 'torso'),
          GetLen(sk, 'left_shoulder', 'neck'),
          GetLen(sk, 'right_shoulder', 'neck'),
          GetLen(sk, 'left_elbow', 'left_shoulder'),
          GetLen(sk, 'right_elbow', 'right_shoulder'),
          GetLen(sk, 'left_hand', 'left_elbow'),
          GetLen(sk, 'right_hand', 'right_elbow'),
          GetLen(sk, 'left_hip', 'torso'),
          GetLen(sk, 'right_hip', 'torso'),
          GetLen(sk, 'left_knee', 'left_hip'),
          GetLen(sk, 'right_knee', 'right_hip'),
          GetLen(sk, 'left_foot', 'left_knee'),
          GetLen(sk, 'right_foot', 'right_knee')
        ]

def median(items):
    items = list(sorted(items))
    return items[len(items)/2]


names = 'neck upper-torso left-shoulder right-shoulder left-upper-arm right-upper-arm left-forearm right-forearm left-pelvis right-pelvis left-thigh right-thigh left-lower-leg right-lower-leg'.split()

for name,limbLens in zip(names, zip(*list(Lengths()))): # transpose
    totalNum = len(limbLens)
    limbLens = [l for l in limbLens if l is not None] # filter out unknown values
    
    medi = median(limbLens)
    mean = sum(limbLens)/len(limbLens)
    meanVariance = sum((item - mean)**2 for item in limbLens)/(len(limbLens))
    medianVariance = sum((item - medi)**2 for item in limbLens)/(len(limbLens))
    print '%17s min: %0.5f  median: %0.5f  max: %0.5f | variance: %0.5f  median-variance: %0.5f  # invalid %3i'%(
        name,min(limbLens),medi,max(limbLens), meanVariance, medianVariance, (totalNum-len(limbLens))
    )

