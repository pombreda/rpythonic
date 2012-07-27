#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("ogretest.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

if 1:
	print '_'*80
	print h.classes['InstancedGeometry']
	print h.classes['InstancedGeometry']._public_typedefs

	print '_'*80
	for meth in h.classes['InstancedGeometry']['methods']['public']:
		print '	method name:', meth['name']
		print '	method returns:', meth['returns']

print( h.typedefs )
