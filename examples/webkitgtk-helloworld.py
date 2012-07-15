#!/usr/bin/pypy

import os, sys, time, ctypes
import webkitgtk as webkit
gtk = glib = webkit	# webkit links to gtk and glib
#glib.g_type_init()	# not required, gtk init should do this
gtk.init()


################### java script core ##################
a = 400
b = 20.0
ctx = webkit.JSGlobalContextCreate( None )
body = webkit.JSStringCreateWithUTF8CString( 'return %s + %s' %(a,b) )
func = webkit.JSObjectMakeFunction(
	ctx,
	None,	# name
	0,		# number of arguments
	None,	# argument names
	body,	# function body
	None,	# source url
	1,		# starting line number
	None	# exception object
)
print(func)
result = webkit.JSObjectCallAsFunction( ctx, func, None, 0, None, None )
value = webkit.JSValueToNumber( ctx, result, None )
assert value == a+b
print(result, value)
print('javascriptcore test complete')
################################################


################## Test WebKitGTK ###################
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
frame = view.get_main_frame()

dom = view.get_dom_document()
html = webkit.dom_html_element_get_inner_html( dom )
print( html )


win = gtk.Window()
win.add( view )
win.set_default_size( 320, 240 )
win.show_all()

while True:
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

print('webkit-gtk test complete')






