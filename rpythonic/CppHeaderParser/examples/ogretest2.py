#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try:
    cppHeader = CppHeaderParser.CppHeader("ogretest2.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

if 1:
	print '_'*80
	print cppHeader.classes['ResourceManager']

	print '_'*80
	for meth in cppHeader.classes['ResourceManager'].get_all_methods():	#['methods']['public']:
		print meth['name']

	print '_'*80

	print 'ResourceManager is abstract', cppHeader.classes['ResourceManager']['abstract']
