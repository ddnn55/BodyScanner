#!/usr/bin/env python
import sys
if len(sys.argv) < 3:
  print 'usage: vtk2obj.py [mesh.vtk] [mesh.obj]'

fin = open(sys.argv[1], 'r')
fout = open(sys.argv[2],'w')

numPoints = 0
while True:
  line = fin.readline()
  if line.startswith('POINTS'):
    numPoints = int(line.split()[1])
    break

for i in range(numPoints):
  fout.write('v '+fin.readline())

while True:
  if fin.readline().startswith('POLYGONS'):
    break

triangles = []
while True:
  line = fin.readline()
  if not line: break
  parts = line.split()
  indices = [int(i)+1 for i in parts[1:]]
  if len(indices) == 3:
    triangles.append(indices)


V = []
for ijk in triangles:
  V += ijk 

def computeO(V):
  O = [-1]*len(V)
  edges = {}
  for t,(i,j,k) in enumerate(triangles):
    for _c,(u,v) in enumerate([(j,k), (k,i), (i,j)]):
       c = t*3+_c
       o = edges.get((u,v), None)
       if o: # flipped
         O[c] = o
         O[o] = c
         del edges[(u,v)]
       o = edges.get((v,u), None)
       if o: # normal
         O[c] = o
         O[o] = c
         del edges[(v,u)]
       else:
         edges[(u,v)] = c
  print 'percent edges',sum(1 for o in O if o == -1)/float(len(O))
  return O



def n(c):
  if c%3 == 2: return c-2
  else: return c+1
def p(c):
  if c%3 == 0: return c+2
  else: return c-1

class Orient(object):
  def __init__(self, V, O):
    self.M = [False]*len(triangles)
    self.V = V
    self.O = O
    self.F = []
  def run(self):
    for t in range(len(self.V)/3):
      if not self.M[t]:
        self.fix(t)
        while self.F:
          t = self.F.pop()
          self.fix(t)
  
  def fix(self, t):
    V,O,M = self.V,self.O,self.M
    if not M[t]:
      M[t] = True
      for c in range(t*3, (t+1)*3):
        o = O[c]
        if o >= 0 and not M[o//3]:
          #M[o//3] = True
          if V[n(o)] == V[n(c)] and V[p(o)] == V[p(c)]:
            V[n(o)],V[p(o)] = V[p(o)],V[n(o)] # flip
            #O[O[n(o)]],O[O[p(o)]] = p(o),n(o)
            O[n(o)],O[p(o)] = O[p(o)],O[n(o)]
            self.F.append(o//3)
          elif V[n(o)] == V[p(c)] and V[p(o)] == V[n(c)]:
            pass # what we want
            self.F.append(o//3)
          else:
            # who knows why these are connected
            self.O[c] = -1            

O = computeO(V)
Orient(V,O).run()

for c in range(0, len(V), 3):
    fout.write('f %i %i %i\n'%(V[c+2],V[c+1],V[c+0]))

fout.close()
fin.close()

