#!/usr/bin/pypy
import os, sys, time, ctypes
import libgio as gio
glib = gio
gio.g_type_init()

file = gio.file_new_for_path( '/tmp' )
info = gio.file_query_info(file, "standard::*", gio.G_FILE_QUERY_INFO_NONE, None, None)
print(info)

print('info-name', info.get_name() )
print('info-size', info.get_size() )
print('info-icon', info.get_icon() )
print('info-content-type', info.get_content_type() )

############## test Glib ################
def Variant( *args ):
	'''
	can call this with multiple or single arguments,
	if a single argument and its wrapped in a tuple,
	then wrap Variant in a tuple. (required for remote method calls)
	'''
	if len(args) == 1:
		a = args[0]
		wrap = False
		if isinstance(a, tuple):
			wrap = True
			a = a[0]
		if isinstance(a, basestring):
			var = glib.variant_new_string( a )
		if wrap:
			array = ( ctypes.POINTER(glib.GVariant.CSTRUCT)*1 )( var.POINTER )
			var = glib.variant_new_tuple( array, 1 )
		return var

	elif len(args) > 1:
		gargs = ( ctypes.POINTER(glib.GVariant.CSTRUCT)*len(args) )()
		if type(args[0]) == str:
			for i,a in enumerate(args):
				gargs[i] = glib.variant_new_string( a ).POINTER

		return glib.variant_new_array( None, gargs, len(args) )

hello = 'hello'
world = 'world'
var = Variant( hello, world )
assert var.n_children() == 2
a = var.get_child_value(0)
b = var.get_child_value(1)
assert a.get_size() == len(hello)+1
assert b.get_size() == len(world)+1

s = a.get_string()
for i,char in enumerate(hello):
	assert s[i] == char

s = b.get_string()
for i,char in enumerate(world):
	assert s[i] == char

t = var.get_type()
assert t.is_array() and t.is_container() and t.is_definite()
assert t.get_string_length() == 2
s = t.peek_string()
assert s[0] == 'a'		# array
assert s[1] == 's'		# of string

############### test bus ##############
bus = gio.bus_get_sync( gio.G_BUS_TYPE_SESSION, None )
proxy = gio.dbus_proxy_new_sync(
	bus,
	gio.G_DBUS_PROXY_FLAGS_NONE,
	None,
	'org.freedesktop.DBus',
	'/org/freedesktop/DBus',
	'org.freedesktop.DBus',
	None,
	None
)
error = glib.GError()
res = proxy.call_sync(
	'ListNames', 
	None, 
	gio.G_DBUS_CALL_FLAGS_NO_AUTO_START,
	500,
	None,
	error
)
assert not error.code
## unbox ##
assert res.n_children() == 1
assert res.get_type().peek_string() == '(as)'
a = res.get_child_value(0)
for i in range( a.n_children() ):
	name = a.get_child_value(i).get_string()
	if not name.startswith(':'):		#ignore private
		print( name )

#########################################
##Creates a proxy for accessing interface_name on the remote object at object_path owned by name at connection and synchronously loads D-Bus properties unless the G_DBUS_PROXY_FLAGS_DO_NOT_LOAD_PROPERTIES flag is used. ##
## possible issue with Properties: 'org.freedesktop.DBus.Properties'
proxy = gio.dbus_proxy_new_sync(
	bus,		# GIOStream
	gio.G_DBUS_PROXY_FLAGS_NONE,
	None,	# info : ctypes.POINTER(_GDBusInterfaceInfo)
	'org.cinnamon.insync',	# name
	'/org/cinnamon/insync',	# object path
	'org.freedesktop.DBus',	# interface name
	None,	# ctypes.POINTER(_GCancellable))
	None	# ctypes.POINTER(ctypes.POINTER(_GError)))
)
error = ctypes.pointer(ctypes.c_void_p())
arg = Variant( tuple(['ctypes test']) )
arg.ref_sink()
res = proxy.call_sync(
	'hello_world', 
	arg, 
	gio.G_DBUS_CALL_FLAGS_NO_AUTO_START,
	500,
	None,
	error
)
error = ctypes.cast( error, ctypes.POINTER(glib.GError.CSTRUCT) )
assert not error.contents.code

print('ctypes gio test complete')



