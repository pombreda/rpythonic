#!/usr/bin/pypy

import os, sys, time, ctypes
import webkitgtk as webkit
gtk = webkit
gtk.init()


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
print('webkit test complete')

############ Test WebKitGTK #############
view = webkit.web_view_new()
print(view)

#view.load_string('hello world', "text/html", "iso-8859-15", "mytitle")
view.load_uri( 'http://google.com' )
frame = view.get_main_frame()

win = gtk.Window()
win.add( view )
win.set_default_size( 320, 240 )
win.show_all()

while True:
	if gtk.gtk_events_pending():
		gtk.gtk_main_iteration()

print('webkit-gtk test complete')

