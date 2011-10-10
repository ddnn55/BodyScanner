import os,sys,pickle,pcd
from skeleton import Skeleton
from vec import *

# python Canonicalize.py ../recordings/manohar/manohar_00005.pcd ../recordings/manohar/manohar_skeleton_00005.yaml canon5.obj
def main():
    assert len(sys.argv) >= 4
    pcdFile = sys.argv[1]
    yamlFile = sys.argv[2]
    outputFile = sys.argv[3]
    if len(sys.argv) == 5:
        offset = float(sys.argv[4])/2 # for displaying several together, non overlapping
    else:
        offset = 0
    
    assert pcdFile.endswith('pcd')
    assert yamlFile.endswith('yaml')
    assert outputFile.endswith('obj')
    weightFile = 'temp-weights'
    
    cmd = '../segmentation/build/sharpSegmentation %s %s %s'%(pcdFile,yamlFile,weightFile)
    assert os.system(cmd) == 0
    
    skel = Skeleton.fromFunnyYaml(yamlFile, relative=False)
    # save it so that they all use the same skel
    canonFile = 'canon.pickle'
    if os.path.exists(canonFile):
        canon = pickle.load(open(canonFile,'r'))
    else:
        canon = skel.Duplicate()
        canon.Canonicalize()
        #canon.sk = None #safe?
        pickle.dump(canon, open(canonFile,'w'))
    
    points = list(pcd.read(pcdFile))
    weights = ReadWeights(weightFile)
    
    I = ((1,0,0),(0,1,0),(0,0,1))
    newPoints = [[offset,0,0,r,g,b] for (_,_,_,r,g,b) in points]
    
    if True: # the right code
        for jointName,skin in weights.iteritems():
            src = Frame(skel.GetRot(jointName), skel.GetPos(jointName))
            dst = Frame(I, canon.GetPos(jointName))
            print jointName,src.o,dst.o
            for index,weight in skin:
                plusEq(newPoints[index], mult(absolute(dst, relative(src, points[index])), weight))
    else: # tesing code
        j = 0
        for jointName,skin in weights.iteritems():
            src = Frame(skel.GetRot(jointName), skel.GetPos(jointName))
            dst = Frame(I, canon.GetPos(jointName))
            print jointName,src.o,dst.o
            for index,weight in skin:
                plusEq(newPoints[index], relative(src, points[index]))
                plusEq(newPoints[index], (j,0,0))
            j += 0.5
        
    
    f = open(outputFile, 'w')
    for row in newPoints:
        if row[0] == row[0]:
            f.write('v %f %f %f %f %f %f\n'%tuple(row))
    f.close()
    print 'output to',outputFile

def ReadWeights(weightFile):
    weights = {}
    currentJoint = None
    f = open(weightFile, 'r')
    line = f.readline()
    while line:
        if line.startswith('b'):
            jointName = line[2:].strip()
            currentJoint = weights[jointName] = []
        else:
            parts = line.split()
            index,weight = int(parts[0]),float(parts[1])
            currentJoint.append((index,weight))
        line = f.readline()
    return weights


if __name__ == '__main__':
    main()

