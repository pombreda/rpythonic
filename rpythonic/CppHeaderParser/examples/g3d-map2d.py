#!/usr/bin/python
import sys
sys.path = ["../"] + sys.path
import CppHeaderParser
try: h = CppHeaderParser.CppHeader("g3d-map2d.h")
except CppHeaderParser.CppParseError,  e:
    print e
    sys.exit(1)


for name in h.classes:
	print '_'*80
	k = h.classes[name]
	print k
	if 'internal' in k: print 'internal:', k['internal']
	if 'template_typename' in k: print 'template typename:', k['template_typename']
	if 'template_info' in k: print 'template info:', k['template_info']
	if 'template' in k: print 'template:', k['template']
	if 'template_raw' in k: print 'template raw:', k['template_raw']

	print( 'namespace', k['namespace'] )

	print '---------------- public methods ----------------'
	for meth in k['methods']['public']:
		print '	method name:', meth['name']
		print '	method returns:', meth['returns']
		if meth['template']: print '		method template!!'
		for param in meth['parameters']:
			print '		param', param['raw_type']
			if 'default' in param:
				print '			default', param['default']
		#print( meth )

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

	print '---------------- public typedefs: ', k._public_typedefs.keys()


print '_'*80
print '############# free functions #############'
for f in h.functions:
	print( f['name'], f['returns'] )
	for p in f['parameters']:
		print('	%s' %p['name'])
		if 'invalid' in p: print( '	INVALID')
	print( f )

print '_'*80
print '############# template classes #############'
print h.template_classes

print '_'*80
print '############# typedefs #############'
print h.typedefs

print '############# typedefs info #############'
for key in h.typedefs_info:
    t = h.typedefs_info[key]
    print '  Typedef:', key
    if 'template' in t and t['template'] in h.template_classes:
        cls = h.template_classes[ t['template'] ]
        print( 'class:', cls )
        typename = t['typename']
        td = h.template_typedefs[ t['template'] ]
        print '    Template:', td
        assert t['template'] in h.template_classes

print '########### Enums #########'
print h.enums

print '########### Namespaces #########'
print h.namespaces
