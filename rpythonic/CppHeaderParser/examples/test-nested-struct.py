#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("test-nested-struct.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

for name in h.classes:
	print '_'*80
	k = h.classes[name]
	print k
	if 'internal' in k: print 'internal:', k['internal']

	print '---------------- public methods ----------------'
	for meth in k['methods']['public']:
		print '	method name:', meth['name']
		print '	method returns:', meth['returns']
	print '---------------- private methods ----------------'
	for meth in k['methods']['private']:
		print '	method name:', meth['name']
		print '	method returns:', meth['returns']

	print '---------------- public properties ----------------'
	for prop in k['properties']['public']:
		print '	prop:', prop['name']
	print '---------------- private properties ----------------'
	for prop in k['properties']['private']:
		print '	prop:', prop['name']


