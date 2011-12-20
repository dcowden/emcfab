import sys
import time
import numpy as np
import time
import brushes_c

from PIL import Image,ImageDraw


def saveImage(array,fname):
	"Save the pixmap to an image, so that we can debug what it looks like"
	#im = Image.frombuffer("1",(self.nX,self.nY),self.p,'raw',"1",0,1);
	a = np.array(array,np.uint8 );
	a *= 255;
	im = Image.fromarray(a,'L');
	im.save(fname,format="bmp");
	del im;
if __name__=='__main__':
	a = np.zeros((5000,5000),dtype=np.int);
	brushes_c.doWrites(a);
	saveImage(a,'c://temp//brushes_c.jpg');
	