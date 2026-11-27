import sys, numpy as np
from PIL import Image
a=np.fromfile(sys.argv[1],np.uint8)[:320*240*4].reshape(240,320,4)  # 0xAARRGGBB little-endian -> bytes B,G,R,A
img=np.dstack([a[:,:,2],a[:,:,1],a[:,:,0]])  # to RGB
Image.fromarray(img).resize((640,480),Image.NEAREST).save(sys.argv[2])
print('wrote',sys.argv[2])
