scale = 100
def read(fname):
    import struct
    f = open(fname, 'r')
    line = f.readline()
    while not line.startswith('DATA'):
        line = f.readline()
    
    line = f.readline()
    while line:
        parts = line.split()
        if parts[0] != 'nan':
            color = float(parts[-1])
            b,g,r,a = struct.unpack('BBBB', struct.pack('f', color))
            yield -float(parts[0])*scale, -float(parts[1])*scale, float(parts[2])*scale, r/255., g/255., b/255.
        line = f.readline()
