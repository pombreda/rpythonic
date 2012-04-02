#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic

if sys.argv[-1].startswith('--output='):
	rpythonic.set_cache( sys.argv[-1].split('=')[-1] )
else:
	rpythonic.set_cache('../examples' )


GINCLUDE = [
	'/usr/include/glib-2.0/',
	'/usr/lib/glib-2.0/include/',
	'/usr/include/cairo/',
	'/usr/include/pango-1.0/',
	'/usr/include/atk-1.0/',
	'/usr/include/gdk-pixbuf-2.0/',
	'/usr/lib/i386-linux-gnu/glib-2.0/include/',		# glibconfig.h
	'/usr/lib/x86_64-linux-gnu/glib-2.0/include/',	# 64bits
]


footer = open( 'gtkfooter.py', 'rb' ).read()

if '--gtk3' in sys.argv:
	rpythonic.wrap(
		'gtk3',		# do not use "gtk" (already used by old pygtk)
		header='/usr/include/gtk-3.0/gtk/gtk.h', 
		library='/usr/lib/libgtk-3.so',
		includes=['/usr/include/gtk-3.0/'] + GINCLUDE,
		#defines=['__G_THREAD_H__', '__G_ASYNCQUEUE_H__'],
		ctypes_footer=footer,
		strip_prefixes = ['GTK_', 'gtk_'],
		insert_headers = ['/usr/include/gtk-3.0/gtk/gtkx.h'],	# required with gtk3 
	)


###############################

GTK2INCLUDE = [
	'/usr/include/gtk-2.0/',
	'/usr/lib/gtk-2.0/include/',						# gdkconfig.h
]

if '--gtk2' in sys.argv:
	rpythonic.wrap( 
		'gtk2', 
		header='/usr/include/gtk-2.0/gtk/gtk.h', 
		includes=GTK2INCLUDE + GINCLUDE, ctypes_footer=footer,
		library= '/usr/lib/libgtk-x11-2.0.so',
		strip_prefixes = ['GTK_', 'gtk_'],
	)



if '--wnck' in sys.argv:
	#sudo apt-get install libwnck-dev
	rpythonic.wrap( 'wnck', 
		defines = ['WNCK_I_KNOW_THIS_IS_UNSTABLE'],
		#includes=[ '/usr/include/libwnck-1.0'] + GTK2INCLUDE + GINCLUDE,
		includes=[ '/usr/include/libwnck-1.0', '/usr/include/gtk-3.0/'] + GINCLUDE,
		header='/usr/include/libwnck-1.0/libwnck/libwnck.h',
		library = '/usr/lib/libwnck-1.so',
		strip_prefixes = ['wnck_'],
	)


if '--gimp' in sys.argv:
	rpythonic.wrap( 'libgimp', 
		header='/usr/include/gimp-2.0/libgimp/gimp.h',
		library = '/usr/lib/libgimp-2.0.so',
		includes = ['/usr/include/gimp-2.0/'] + GINCLUDE,
		strip_prefixes = ['gimp_', 'Gimp', 'GIMP'],
	)

if '--gstreamer' in sys.argv:
	# apt-get install libgstreamer0.10-dev
	rpythonic.wrap( 'libgstreamer', 
		header='/usr/include/gstreamer-0.10/gst/gst.h',
		library = 'libgstreamer-0.10.so',
		includes = ['/usr/include/gstreamer-0.10/', '/usr/include/libxml2'] + GINCLUDE,
		strip_prefixes = ['gst_'],
	)

