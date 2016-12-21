import os
import time
from lindypy import *

with tspace() as ts:
	print('go')
	ts.add('1','2','3')
	print(ts.get('1',str,str))
