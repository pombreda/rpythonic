#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try:
    h = cppHeader = CppHeaderParser.CppHeader("simple.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

print '_'*80
print cppHeader.classes['MyClass']
print cppHeader.classes['MyClass']['forward_declares']

print cppHeader.typedefs

print h.classes['XXX']
for meth in h.classes['XXX']['methods']['public']:
	print( meth )

