#!../../pypy-1.7/bin/pypy
# Dec 2011
import os, sys, ctypes
import python3 as py

print( 'is py inited', py.Py_IsInitialized() )
print('init py')
py.Py_Initialize()
print( 'py ok' )

test = '''
print( 'python in python' )
for i in range(100):
	print( i )
print( 'python in python exit' )
'''

#py.Run_SimpleString( test, ctypes.pointer(py.CompilerFlags(1)) )
#py.Run_SimpleString( test, ctypes.pointer(ctypes.c_void_p()) )
py.PyRun_SimpleString( test )
#py.Run_String( test )
#py.Main(argc,argv)

#py.Finalize()

print( 'host exit' )




