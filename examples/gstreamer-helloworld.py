#!/usr/bin/pypy
#!/usr/bin/python

import os, sys, time, ctypes
import libgstreamer as gst

gcontext = gst.g_main_context_new()
print(gcontext)
#locks = gst.GThreadFunctions()
#gst.g_thread_init( locks )
print( 'g threads ready', gst.g_thread_get_initialized() )


gst.init( ctypes.pointer(ctypes.c_int(0)) )
assert gst.is_initialized()
print('gst version: %s' %gst.version_string())

def on_message( bus, msg ):
	ptr = ctypes.POINTER(ctypes.c_void_p).from_address( msg )
	msg = gst.GstMessage( pointer=ctypes.pointer(ptr), cast=True )
	if msg.type == gst.GST_MESSAGE_EOS: print('end of stream')
	elif msg.type == gst.GST_MESSAGE_ERROR: print('stream error')


def on_sync_message( bus, msg ):
	ptr = ctypes.POINTER(ctypes.c_void_p).from_address( msg )
	msg = gst.GstMessage( pointer=ctypes.pointer(ptr), cast=True )
	print( msg.structure )
	name = gst.structure_get_name( msg.structure )
	print( name )
	#print( 'has name', gst.structure_has_field( msg.structure, 'name' ) )
	if name in ('void', 'GInterface', 'prepare-xwindow-id'):
		src = msg.src	#ctypes.POINTER(_GstObject)
		if src:
			print( 'preparing window...' )
			print( src )
			#gst.child_proxy_set_property( src, 'force-aspect-ratio', True )	# segfaults
			#gst.g_object_set_property( src, 'force-aspect-ratio', True )	# segfaults

player = gst.parse_launch("v4l2src ! autovideosink")
print(player)
bus = player.get_bus()
print(bus)
bus.add_signal_watch()
bus.enable_sync_message_emission()
bus.connect("message", on_message)
bus.connect("sync-message::element", on_sync_message)

player.set_state(gst.GST_STATE_PLAYING)

#loop = gst.g_main_loop_new( gcontext )
#print('running', loop)
#ctx = loop.
#loop.run()	# this works but can not quit

while True:	#gst.g_main_context_pending( gcontext ):
	gst.g_main_context_iteration(
		gcontext,
		True,	# blocking
	)

print('exit')
