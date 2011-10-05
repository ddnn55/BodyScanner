class Skeleton(object):
    scale = 100
    # parse and convert to relative coordinates
    def __init__(self, sk):
        self.sk = sk
        self.pos = {} # cache
        
        self.torso = self.GetPos('torso')
        self.head = self.GetPosRel('head', 'neck')
        self.neck = self.GetPosRel('neck', 'torso')

        self.leftShoulder = self.GetPosRel('left_shoulder', 'neck')
        self.rightShoulder = self.GetPosRel('right_shoulder', 'neck')

        self.leftElbow = self.GetPosRel('left_elbow', 'left_shoulder')
        self.rightElbow = self.GetPosRel('right_elbow', 'right_shoulder')

        self.leftHand = self.GetPosRel('left_hand', 'left_elbow')
        self.rightHand = self.GetPosRel('right_hand', 'right_elbow')

        self.leftHip = self.GetPosRel('left_hip', 'torso')
        self.rightHip = self.GetPosRel('right_hip', 'torso')

        self.leftKnee = self.GetPosRel('left_knee', 'left_hip')
        self.rightKnee = self.GetPosRel('right_knee', 'right_hip')
        self.leftFoot = self.GetPosRel('left_foot', 'left_knee')
        self.rightFoot = self.GetPosRel('right_foot', 'right_knee')

    def GetPos(self, jointName):
        pos = self.pos.get(jointName, None)
        if pos is not None: # cache the parse/transform
            return pos
        else:
            # maybe the _1 indicates that it tracks several people?
            joint = self.sk.get('/'+jointName+'_1',None)
            if joint is None:
                return None
            else:
                tr = joint['translation']
                return ( # shuffle so that the skelton is upright, remember to do this transform on the mesh too
                    float(tr[1].replace('y -- ', ''))*self.scale,
                    float(tr[2].replace('z -- ', ''))*self.scale,
                    float(tr[0].replace('x -- ', ''))*self.scale
                )

    def GetPosRel(self, jointName, parentName):
        j = self.GetPos(jointName)
        p = self.GetPos(parentName)
        if j is None or p is None:
            return None
        else:
            return (j[0]-p[0], j[1]-p[1], j[2]-p[2])
    
    def GetBone(self, src, dst):
        return self.GetPos(src),self.GetPos(dst)
    
    def Bones(self):
        return {
            'T'    : self.GetBone('torso', 'neck'),
            'LS2E' : self.GetBone('left_shoulder', 'left_elbow'),
            'LE2H' : self.GetBone('left_elbow', 'left_hand'),
            'N2H'  : self.GetBone('neck', 'head'),
            'RS2E' : self.GetBone('right_shoulder', 'right_elbow'),
            'RE2H' : self.GetBone('right_elbow', 'right_hand'),
            'LH2K' : self.GetBone('left_hip', 'left_knee'),
            'RH2K' : self.GetBone('right_hip', 'right_knee'),
            'LK2F' : self.GetBone('left_knee', 'left_foot'),
            'RK2F' : self.GetBone('right_knee', 'right_foot')
        }        

