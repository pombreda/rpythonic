#!/usr/bin/python
# updated may19th 2011
import os,sys, time

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
gtk = rpythonic.module( 'gtk' )
assert gtk

gtk.init()
win = gtk.window_new( gtk.GTK_WINDOW_TOPLEVEL )
print( win )

#win.set_title( 'ctypes_ooapi_gtk_test')	# segfaults, name too long?
win.set_title('hello world')
win.set_default_size( 320, 240 )

def exit(*args): gtk.main_quit()
win.connect( 'destroy', exit )

frame = gtk.frame_new('hello world')
win.add( frame )

button = gtk.button_new_with_label("test")
frame.add( button )

def callback(*args): print('python callback')
button.connect( 'clicked', callback )

win.show_all()
gtk.main()

print('exit')

