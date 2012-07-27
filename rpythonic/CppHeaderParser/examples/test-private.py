#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("test-private.h")
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
		for param in meth['parameters']:
			print '		param', param['raw_type']
			if 'default' in param:
				print '			default', param['default']


	print '---------------- private methods ----------------'
	for meth in k['methods']['private']:
		print '	method name:', meth['name']
		print '	method returns:', meth['returns']

	print '---------------- public properties ----------------'
	for prop in k['properties']['public']:
		print '	prop:', prop['name']
	print '---------------- private properties ----------------'
	for prop in k['properties']['private']:
		print '	prop:', prop['name']; print( prop )

print '_'*80
assert h.functions
for f in h.functions: print( f['name'], f['returns'] )

