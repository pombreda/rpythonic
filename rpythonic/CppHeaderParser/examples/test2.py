#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try:
    cppHeader = CppHeaderParser.CppHeader("SampleClass.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)


# by hart
print 'global typedefs:'
print cppHeader.typedefs


#print cppHeader.classes['ThetaClass']
print '_'*80
#print cppHeader.classes['ThetaClass2']

print cppHeader.structs
for s in cppHeader.structs:
	print '\t', s
	for x in cppHeader.structs[s]['fields']: print '\t\t', x['type'], x['name']

print cppHeader.namespaces

print cppHeader.classes['FixMe']

print cppHeader.classes['SubClassFixMe']

print cppHeader.classes['StillAbstract']


print '_'*80
print cppHeader.classes['ThetaClass2']

print '_'*80
print cppHeader.classes['ThetaClass']

for cls in cppHeader.classes.values():
	print cls['name'], 'methods:', len(cls.get_all_methods())

