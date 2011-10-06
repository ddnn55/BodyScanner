#!/usr/bin/env python

def main():
    import os, yaml

    l = os.listdir('david2/pcd')
    y = yaml.load(open('david2/david2_skeleton.yaml', 'r'))
    yk = y.keys()
    yk.sort()

    def findMatches():
        scanTimes = []
        for f in l:
            t = float(f.replace('david2_','').replace('.pcd', ''))
            scanTimes.append((t,f))
        
        for skt in yk:
            minDiff = 1e100
            bestFile = None
            for t,f in scanTimes:
                if t < skt and abs(skt-t) < minDiff:
                    minDiff = abs(skt-t)
                    bestFile = f
            yield (skt,bestFile) # mapping from sks

    for row in findMatches():
        print row,','


if __name__ == '__main__':
    main()
