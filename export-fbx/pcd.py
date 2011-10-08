scale = 1
def read(fname):
    import struct
    f = open(fname, 'r')
    line = f.readline()
    while not line.startswith('DATA'):
        line = f.readline()
    
    if 'binary' in line:
        row = f.read(16)
        while len(row) == 16:
            x,y,z,b,g,r,a = struct.unpack('fffBBBB', row)
            yield -x*scale,-y*scale,-z*scale, r/255.,g/255.,b/255.
            row = f.read(16)
    else: # ascii
        line = f.readline()
        while line:
            parts = line.split()
            if parts[0] != 'nan':
                color = float(parts[-1])
                b,g,r,a = struct.unpack('BBBB', struct.pack('f', color))
                yield -float(parts[0])*scale, -float(parts[1])*scale, float(parts[2])*scale, r/255., g/255., b/255.
            line = f.readline()
