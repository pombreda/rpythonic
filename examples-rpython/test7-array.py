#!/usr/bin/python
import os, sys, array, time, ctypes
sys.path.append('..')
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
################################
rpy = rpythonic.RPython('rpyarraytest')

import pypy.rpython.lltypesystem.rffi as rffi

@rpy.bind( addr=int, length=int )
def test_float( addr, length ):
	a = rffi.cast( rffi.FLOATP, addr )
	for i in range( length ):
		f = rffi.cast( rffi.DOUBLE, a[i] )
		print f

@rpy.bind( addr=int, length=int )
def test( addr, length ):
	a = rffi.cast( rffi.DOUBLEP, addr )
	x = ['<somexml']
	for i in range( length ):
		print a[i]
		x.append( 'attribute="%s"' %a[i] )
	x.append( '>' )
	print( ' '.join(x) )


rpy.cache(refresh=1)

############### testing ##############

#x = array.array('i', [.0, .5])
x = (ctypes.c_double * 2)()
x[0] = .1
x[1] = .99
ptr = ctypes.pointer(x)
test( ctypes.addressof(x), 2 )
print('double test done')

x = (ctypes.c_float * 2)()
x[0] = .66
x[1] = .44
test_float( ctypes.addressof(x), 2 )


print( 'float test done' )


