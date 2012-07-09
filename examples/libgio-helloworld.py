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
def _python_simple_type_to_gvariant( arg ):
		'''
		TODO other int types, byte and bytearray
		'''
		if isinstance(arg, basestring):
			var = glib.variant_new_string( arg )
		elif isinstance(arg, float):
			var = glib.g_variant_new_double( arg )
		elif isinstance(arg, int):
			var = glib.g_variant_new_int32( arg )
		elif isinstance(arg, bool):
			var = glib.g_variant_new_boolean( arg )

		elif isinstance(arg, list):	# assume array (RECURSIVE)
			array = ( ctypes.POINTER(glib.GVariant.CSTRUCT)*len(arg) )()
			for i,a in enumerate(arg): array[ i ] = _python_simple_type_to_gvariant( a ).POINTER
			return glib.variant_new_array( None, array, len(arg) )

		else:
			raise NotImplementedError

		return var

def Variant( *args, **kw ):
	'''
	can call this with multiple or single arguments,
	Single Argument:
		. if argument is a tuple, then wrap Variant in a tuple. (required for remote method calls)
		. if argument is a tuple, and container a sub-tuple, wrap return in tuple (required for remote properties "Set")
	'''
	if len(args) == 1:
		sub = args[0]
		array = None
		if isinstance(sub, tuple) or 'wrap_tuple' in kw:
			if isinstance(sub, tuple): _len = len(sub)
			else: _len = 1; sub = [sub]
			array = ( ctypes.POINTER(glib.GVariant.CSTRUCT)*_len )()
		if array:
			for i,arg in enumerate(sub): array[ i ] = _python_simple_type_to_gvariant( arg ).POINTER
			return glib.variant_new_tuple( array, len(sub) )
		else:
			return _python_simple_type_to_gvariant( sub )


	elif len(args) > 1:
		## check if all the same - if not then wrap in tuple ##
		_types = [type(a) for a in args]
		is_array = _types.count(_types[0]) == len(_types)
		if 'wrap_tuple' in kw: is_array = False	# force over-ride

		array = ( ctypes.POINTER(glib.GVariant.CSTRUCT)*len(args) )()
		for i,a in enumerate(args):
				array[ i ] = _python_simple_type_to_gvariant( a ).POINTER
		if is_array:
			return glib.variant_new_array( None, array, len(args) )
		else:
			return glib.variant_new_tuple( array, len(args) )


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

arg = Variant( tuple(['ctypes test']) )					# these are the same
arg = Variant( 'ctypes test', wrap_tuple=True )	# these are the same

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


################ TODO FIX ME : Dbus Props ################
## dbus props are slower than direct method calls ##
#http://dbus.freedesktop.org/doc/dbus-specification.html#standard-interfaces
proxy = gio.dbus_proxy_new_sync(
	bus,		# GIOStream
	gio.G_DBUS_PROXY_FLAGS_NONE,
	None,	# info : ctypes.POINTER(_GDBusInterfaceInfo)
	'org.cinnamon.insync',	# name
	'/org/cinnamon/insync',	# object path
	'org.freedesktop.DBus.Properties',	# interface name
	None,	# ctypes.POINTER(_GCancellable))
	None	# ctypes.POINTER(ctypes.POINTER(_GError)))
)
error = ctypes.pointer(ctypes.c_void_p())

arg = Variant(
	'org.cinnamon.insync',	# object interface
	'recent_changes',			# property name
	['ctypes-prop1', 'ctypes-prop2', 'ctypes-prop3']
)
arg.ref_sink()
res = proxy.call_sync(
	'Set', 
	arg, 
	gio.G_DBUS_CALL_FLAGS_NO_AUTO_START,
	500,
	None,
	error
)
error = ctypes.cast( error, ctypes.POINTER(glib.GError.CSTRUCT) )
assert not error.contents.code


print('ctypes gio test complete')



