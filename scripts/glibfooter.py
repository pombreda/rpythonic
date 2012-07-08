
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

