#!/usr/bin/pypy

import os, sys, time, ctypes
import webkitgtk as webkit
gtk = glib = webkit	# webkit links to gtk and glib
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
view = webkit.web_view_new()
print(view)

''' TODO
settings = webkit.web_settings_new()

class _data( ctypes.Structure ):
	_fields_ = [
		('padding', ctypes.c_uint),
	]

class _struct( ctypes.Structure ):
	_fields_ = [
		('type', ctypes.c_uint),
		('data', (_data*1)),
	]
ptr = ctypes.pointer( _struct() )

print('gtype', glib.G_TYPE_BOOLEAN)

gval = glib.g_value_init( ptr, glib.G_TYPE_BOOLEAN )

#raise None

glib.g_value_set_boolean( gval, True )
#assert glib.g_value_get_boolean( gval )


glib.g_object_class_find_property( gclass, 'enable-webgl' )

glib.g_object_set_property( settings, 'enable-webgl', True )
gval = glib.g_object_get_property( settings, 'enable-webgl' )
view.set_settings( settings )
'''

#view.load_string('hello world', "text/html", "iso-8859-15", "mytitle")
view.load_uri( 'http://google.com' )
#view.load_uri( 'http://get.webgl.org/')	# webkit still not built by default with webgl
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






