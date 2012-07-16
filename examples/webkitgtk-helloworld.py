#!/usr/bin/pypy

import os, sys, time, ctypes
import webkitgtk as webkit
gtk = glib = webkit	# webkit links to gtk and glib
#glib.g_type_init()	# not required, gtk init should do this
gtk.init()


################## Test WebKitGTK ###################
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

view = webkit.webkit_web_view_new()	#WebKitWebView()
print(view)


settings = webkit.web_settings_new()
for prop in 'enable-webaudio enable-file-access-from-file-uris enable-universal-access-from-file-uris enable-developer-extras enable-accelerated-compositing enable-webgl'.split():
	gval = glib.GValue(True)
	glib.g_object_set_property( settings, prop, gval )
view.set_settings( settings )
#view.load_uri( 'http://get.webgl.org/')	# webkit still not built by default with webgl

#view.load_string('hello world', "text/html", "iso-8859-15", "mytitle")
view.load_uri( 'http://pyppet.blogspot.com' )



win = gtk.Window()
root = gtk.VBox()
win.add( root )

header = gtk.HBox()
root.pack_start( header, expand=False )

button = gtk.Button('print html')
button.connect('clicked', lambda b: get_html() )
header.pack_start( button )

button = gtk.Button('print 1+2')
button.connect('clicked', lambda b: call_javascript('1+2') )
header.pack_start( button )


root.pack_start( view, expand=True )

win.set_default_size( 320, 240 )
win.show_all()

while True:
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

print('webkit-gtk test complete')






