
import os,sys,yaml
sys.path.append('../export-fbx')
from skeleton import Skeleton
import pcd
from math import sqrt

def subtract(a,b):
    return a[0]-b[0],a[1]-b[1],a[2]-b[2]

def crossNormSqr(a,b):
    return ((a[1]*b[2] - b[1]*a[2])**2.0
          + (a[0]*b[2] - b[0]*a[2])**2.0
          + (a[0]*b[1] - b[0]*a[1])**2.0)

def dot(a,b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def blend(cA,cB,t):
    return cA[0] + (cB[0]-cA[0])*t,\
           cA[1] + (cB[1]-cA[1])*t,\
           cA[2] + (cB[2]-cA[2])*t,

def main():
    points = list(pcd.read('david2/pcd/david2_1317518872.840330800.pcd'))
    sk = Skeleton(boneData())
    skBones = sk.Bones()
    segments = dict((key,[]) for key in skBones.keys()) # setup
    for p in points:
        minDist = 1e20 # big
        bestBone = 'T' # torso by default
        for bName,bone in skBones.iteritems(): # for all bones
            src,dst = bone
            boneDir = subtract(dst, src)
            offset = subtract(p, src)
            dist = crossNormSqr(boneDir,offset)/dot(offset,offset)
            if dist < minDist:
                minDist = dist
                bestBone = bName
        
        if sqrt(minDist) < 10:
            segments[bestBone].append(p)
    
    f = open('segmented.obj', 'w')
    for segName,points in segments.iteritems():
        print segName, len(points)
        if segName in ('LE2H', 'RE2H', 'LK2F', 'RK2F'):
            color = (1,0,0)
        elif segName in ('LS2E', 'RS2E', 'LH2K', 'RH2K'):
            color = (0,1,0)
        elif segName == 'T':
            color = (0,0,1)
        elif segName == 'N2H':
            color = (0,1,1)
        for p in points:
            col = blend(p[3:], color, 0.25) # tint
            f.write('v %f %f %f %f %f %f\n'%(p[0],p[1],p[2],col[0],col[1],col[2]))

def boneData():
  return yaml.load("""
all:
  /left_hand_1:
    translation:
      - x -- 2.55093038006
      - y -- 0.746679353006
      - z -- 0.0711659839038
  /right_hand_1:
    translation:
      - x -- 2.49713129046
      - y -- -0.364766096082
      - z -- 0.0849115352327
  /left_knee_1:
    translation:
      - x -- 2.63769164562
      - y -- 0.290046279361
      - z -- -0.442379928047
  /left_shoulder_1:
    translation:
      - x -- 2.58867075378
      - y -- 0.327713915459
      - z -- 0.354788586982
  /right_elbow_1:
    translation:
      - x -- 2.65148131201
      - y -- -0.155828990425
      - z -- 0.206661189524
  /head_1:
    translation:
      - x -- 2.52620498714
      - y -- 0.197698793431
      - z -- 0.571711912138
  /left_hip_1:
    translation:
      - x -- 2.66240442744
      - y -- 0.256924666034
      - z -- -0.039866007436
  /left_foot_1:
    translation:
      - x -- 2.70167755327
      - y -- 0.323263433339
      - z -- -0.789495641599
  /torso_1:
    translation:
      - x -- 2.61050045669
      - y -- 0.172315024085
      - z -- 0.163705987267
  /left_elbow_1:
    translation:
      - x -- 2.68997602208
      - y -- 0.514406365212
      - z -- 0.167874945821
  /right_foot_1:
    translation:
      - x -- 2.46705231607
      - y -- -0.048383241478
      - z -- -0.760460233869
  /right_shoulder_1:
    translation:
      - x -- 2.55596896059
      - y -- 0.0459524695006
      - z -- 0.369136119216
  /neck_1:
    translation:
      - x -- 2.57231997926
      - y -- 0.186833194388
      - z -- 0.361962353099
  /right_knee_1:
    translation:
      - x -- 2.5432078558
      - y -- 0.0250231375769
      - z -- -0.415162356864
  /right_hip_1:
    translation:
      - x -- 2.64807263743
      - y -- 0.0712186807718
      - z -- -0.0269647001359
""")['all']


if __name__ == '__main__':
  main()
