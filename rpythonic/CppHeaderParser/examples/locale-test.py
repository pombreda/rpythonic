#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("locale-test.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

if 1:
	print '_'*80
	print h.classes['locale']
	print h.classes['locale']._public_typedefs

	print '_'*80
	for meth in h.classes['locale']['methods']['public']:
		print '	method name:', meth['name']
		print '	method returns:', meth['returns']

print( h.typedefs )
