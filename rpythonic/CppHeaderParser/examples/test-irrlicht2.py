#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("test-irrlicht2.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)

assert h.classes

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
for f in h.functions: print( f['name'], f['returns'] )

print '_'*80
print h.template_classes

print '_'*80
print h.typedefs
for key in h.typedefs_info:
    t = h.typedefs_info[key]
    print '  Typedef:', key
    if 'template' in t:
        cls = h.template_classes[ t['template'] ]
        print( 'class:', cls )
        typename = t['typename']
        td = h.template_typedefs[ t['template'] ]
        print '    Template:', td
        assert t['template'] in h.template_classes



