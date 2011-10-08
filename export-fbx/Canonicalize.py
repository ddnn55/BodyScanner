import os,sys,pcd
from skeleton import Skeleton
from vec import *

# python Canonicalize.py ../recordings/manohar/manohar_00005.pcd ../recordings/manohar/manohar_skeleton_00005.yaml canon5.obj
def main():
    assert len(sys.argv) == 4
    pcdFile = sys.argv[1]
    yamlFile = sys.argv[2]
    outputFile = sys.argv[3]
    assert pcdFile.endswith('pcd')
    assert yamlFile.endswith('yaml')
    assert outputFile.endswith('obj')
    weightFile = 'temp-weights'
    
    cmd = '../segmentation/build/sharpSegmentation %s %s %s'%(pcdFile,yamlFile,weightFile)
    assert os.system(cmd) == 0
    
    skel = Skeleton.fromFunnyYaml(yamlFile)
    canon = skel.Duplicate()
    canon.Canonicalize()
    
    points = list(pcd.read(pcdFile))
    weights = ReadWeights(weightFile)
    
    newPoints = [[0,0,0,r,g,b] for (_,_,_,r,g,b) in points]
    for jointName,skin in weights.iteritems():
        # just translation for the moment
        src = Frame((1,0,0),(0,1,0),(0,0,1), getattr(skel,jointName))
        dst = Frame((1,0,0),(0,1,0),(0,0,1), getattr(canon,jointName))
        print jointName,src.o,dst.o
        for index,weight in skin:
            plusEq(newPoints[index], mult(absolute(dst, relative(src, points[index])), weight))
    
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

