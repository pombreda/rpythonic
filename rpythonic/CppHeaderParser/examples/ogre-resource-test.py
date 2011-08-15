#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("ogre-resource-test.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

if 1:
	print '_'*80
	print h.classes['Resource']
	print h.classes['Resource']._public_enums
	#print h.classes['Resource']['enums']
	print '_'*80
	print h.classes['Resource::Listener']
	print h.classes['Resource::Listener']._public_enums
print '_'*80
print( h.global_enums )
print( h.enums )

