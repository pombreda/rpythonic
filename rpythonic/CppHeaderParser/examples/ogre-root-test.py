#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("ogre-root-test.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

for name in h.classes:
	print '_'*80
	k = h.classes[name]
	print k
	print 'internal:', 'internal' in k

	print '---------------- public methods ----------------'
	for meth in k['methods']['public']:
		print '	method name:', meth['name']
		for key in meth:
			if key.startswith('returns'):
				print '	method %s:' %key, meth[key]
		if 'initialise' in meth['name']:
			#print meth
			for p in meth['parameters']:
				p.pop('method')
				print('-'*80)
				if p['unresolved']: print( p['name'], p['type'], p['raw_type'] )
				print('-'*80)
				print p
		print('_'*80)
		#if meth['returns'] == 'Real': assert 0

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


