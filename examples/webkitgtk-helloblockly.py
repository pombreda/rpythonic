#!/usr/bin/pypy

import os, sys, time, ctypes
import webkitgtk as webkit
gtk = glib = webkit	# webkit links to gtk and glib
#glib.g_type_init()	# not required, gtk init should do this
gtk.init()


def get_html():
	dom = view.get_dom_document()
	html = webkit.dom_html_element_get_inner_html( dom )
	print( html )
	return html

def call_javascript( script ):
	'''
	return the result of the script
	'''
	#view.execute_script('document.title=document.documentElement.innerHTML;')
	view.execute_script('document.title=%s;' %script)
	frame = view.get_main_frame()
	result = frame.get_title()
	print(result)
	return result

def execute_python( script ): exec( script )


################## Test WebKitGTK ###################
view = webkit.webkit_web_view_new()
print(view)


settings = webkit.web_settings_new()
for prop in 'enable-webaudio enable-file-access-from-file-uris enable-universal-access-from-file-uris enable-developer-extras enable-accelerated-compositing enable-webgl'.split():
	gval = glib.GValue(True)
	glib.g_object_set_property( settings, prop, gval )
view.set_settings( settings )

view.load_uri( 'file://%s/test-blockly.html'%os.path.abspath('.'))


win = gtk.Window()
root = gtk.VBox()
win.add( root )

header = gtk.HBox()
root.pack_start( header, expand=False )


button = gtk.Button('print html')
button.connect('clicked', lambda b: get_html() )
header.pack_start( button, expand=False )

header.pack_start( gtk.Label() )

button = gtk.Button('print python')
button.connect('clicked', lambda b: call_javascript("Blockly.Generator.workspaceToCode('Python')") )
header.pack_start( button, expand=False )

button = gtk.Button('run python')
button.connect('clicked', lambda b: execute_python(call_javascript("Blockly.Generator.workspaceToCode('Python')")) )
header.pack_start( button, expand=False )


root.pack_start( view, expand=True )

win.set_default_size( 800, 600 )
win.show_all()

while True:
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

print('webkit-gtk blockly test complete')


