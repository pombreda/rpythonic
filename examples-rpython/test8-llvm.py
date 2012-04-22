#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic
################################
rpy = rpythonic.RPython( 'test8', backend='llvm' )

@rpy.bind(a=int,b=int)
def simple_test(a, b):
	c = 0
	while c < 100000*100000:
		c += a + b
	return c

rpy.cache( refresh=1 )
print('CACHED: starting test...')
############### testing ##############
import time
for i in range(10):
	start = time.time()
	a = simple_test( 1, 1 )
	print('end of benchmark:', time.time()-start)
	print('test result:', a)

