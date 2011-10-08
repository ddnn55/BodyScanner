#!/usr/bin/env python
import os

for i in range(0, 170, 20):
  cmd = 'python Canonicalize.py ../recordings/manohar/manohar_%05i.pcd ../recordings/manohar/manohar_skeleton_%05i.yaml canon%05i.obj %f'%(i,i,i,i/5.)
  print cmd
  assert os.system(cmd) == 0

