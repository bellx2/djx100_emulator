import sys, struct, numpy as np
from PIL import Image
fb=np.fromfile(sys.argv[1],np.uint8)[:256*320].reshape(320,256)[:, :240]
clut=[0xFF000000,0xFFFCFCFC,0xFFFF0080,0xFF00FF00,0xFF00C8FF,0xFFFFFF00,0xFFFF0000,0xFF0000FF,0xFF000000,0xFFFFFFFF]
pal=np.zeros((256,3),np.uint8)
for i,c in enumerate(clut): pal[i]=[(c>>16)&255,(c>>8)&255,c&255]
Image.fromarray(pal[fb]).rotate(-90,expand=True).resize((640,480),Image.NEAREST).save(sys.argv[2])
