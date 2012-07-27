
class _nice_callback_args_container_(object):
	'''	(required for pypy)
	wraps args in an object because pypy ctypes creates a weakref to wrap pyobject
	'''
	def __init__(self,args): self.args = args

class _nice_callback_(object):
	def __del__(self): pass		# for some reason this holds a reference to self
	def __init__(self, widget, func, args):
		import inspect
		self.widget = widget
		self.function = func
		self.args = args
		argspec = inspect.getargspec( func )
		self.num_c_args = len(argspec.args)
		self.num_user_args = len(args)

		n = len(argspec.args) - len(args)
		if not inspect.ismethod( func ): n += 1		# if not a bound-method
		self.cfunc_prototype = ctypes.CFUNCTYPE( ctypes.c_void_p, *([ctypes.c_void_p]*n) )
		g_signal_connect_data.change_argument_type( 'c_handler', self.cfunc_prototype )	# ugly workaround

		self.cfunc = self.cfunc_prototype( self.call )

		self.wrapped_args = _nice_callback_args_container_( args )
		userdata = ctypes.pointer( ctypes.py_object(self.wrapped_args) )
		self.userdata = userdata


	def call(self, *args):
		a = [ self.widget ]	# the first argument is always the widget the signal is attached to
		for i,arg in enumerate(args):
			if i == len(args)-1:
				ptr = ctypes.cast( arg, ctypes.POINTER(ctypes.py_object) )
				w = ptr.contents.value
				a += list(w.args)
			elif i:
				a.append( arg )
		self.function( *a )
		return 0	# pypy complains if None is returned


def connect( ptr, name, func, *args ):
	wrapper = _nice_callback_( ptr.pyobject, func, args )
	return g_signal_connect_data( ptr, name, wrapper.cfunc, wrapper.userdata )


################## Charp to Python String ###############
_GLIB_RETURNS_CHARP_ = (
	g_variant_type_peek_string,
	g_variant_get_string,
	g_value_get_string,
)

for func in _GLIB_RETURNS_CHARP_:
	func.return_wrapper = lambda pointer=None: _CHARP2STRING(pointer)



############## test Glib ################
def _python_simple_type_to_gvariant( arg ):
		'''
		TODO other int types, byte and bytearray
		'''
		if isinstance(arg, _basestring):
			var = variant_new_string( arg )
		elif isinstance(arg, float):
			var = g_variant_new_double( arg )
		elif isinstance(arg, int):
			var = g_variant_new_int32( arg )
		elif isinstance(arg, bool):
			var = g_variant_new_boolean( arg )

		elif isinstance(arg, list):	# assume array (RECURSIVE)
			array = ( ctypes.POINTER(GVariant.CSTRUCT)*len(arg) )()
			for i,a in enumerate(arg): array[ i ] = _python_simple_type_to_gvariant( a ).POINTER
			return variant_new_array( None, array, len(arg) )

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
			array = ( ctypes.POINTER(GVariant.CSTRUCT)*_len )()
		if array:
			for i,arg in enumerate(sub): array[ i ] = _python_simple_type_to_gvariant( arg ).POINTER
			return variant_new_tuple( array, len(sub) )
		else:
			return _python_simple_type_to_gvariant( sub )


	elif len(args) > 1:
		## check if all the same - if not then wrap in tuple ##
		_types = [type(a) for a in args]
		is_array = _types.count(_types[0]) == len(_types)
		if 'wrap_tuple' in kw: is_array = False	# force over-ride

		array = ( ctypes.POINTER(GVariant.CSTRUCT)*len(args) )()
		for i,a in enumerate(args):
				array[ i ] = _python_simple_type_to_gvariant( a ).POINTER
		if is_array:
			return variant_new_array( None, array, len(args) )
		else:
			return variant_new_tuple( array, len(args) )

