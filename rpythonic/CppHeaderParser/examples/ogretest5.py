#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("ogretest5.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

for name in h.classes:
	print '_'*80; print name
	klass = h.classes[ name ]
	print 'Parent:', klass['parent']
	print klass._public_typedefs
	print '-'*80
	for meth in klass['methods']['public']:
		print '	method name:', meth['name']
		print '	method returns:', meth['returns']
		print '	method params:'
		for p in meth['parameters']:
			print '		', p['type']

print( '----------------- classes -------------------' )
print( h.classes )
print( '----------------- structs -------------------' )
print( h.structs )
print( '-----------current struct-------------' )
print( h.curStruct )

