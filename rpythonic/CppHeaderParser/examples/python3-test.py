#!/usr/bin/python3
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
h = CppHeaderParser.CppHeader("boost-test1.h")


print( '_'*80 )
print( h.classes['thread_group'] )
print( h.classes['thread_group']._public_typedefs )

print( '_'*80 )
for meth in h.classes['thread_group']['methods']['public']:
	print( '	method name:', meth['name'] )
	print( '	method returns:', meth['returns'] )
	print( '	method params:' )
	for param in meth['parameters']:
		print( '\t\t name:%(name)s type:%(type)s' %param )
		print( '\t\t %s' %param )
	print( '-'*80 )

print( h.typedefs )
