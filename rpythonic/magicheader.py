import os, sys, ctypes, inspect
__os = os
__sys = sys
__inspect = inspect
_list = list
_CTYPES_CDLLS = []	# support loading functions from multiple libraries


PYTHON_RESERVED_KEYWORDS = 'for while in as global with try except lambda return raise if else elif eval exec and not or break continue finally print yield del def class assert from is pass'.split()


IS32BIT = (ctypes.sizeof(ctypes.c_void_p)==4)

_ISPYTHON2 = sys.version_info[0] == 2
if _ISPYTHON2:
	_NULLBYTE = '\0'
	_basestring = basestring
else:
	_NULLBYTE = bytes( chr(0), 'ascii' )
	_basestring = str

def _CHARP2STRING( charp, encoding='utf-8' ):
	b = bytes()
	i = 0
	while True:
		char = charp[ i ]
		if char == _NULLBYTE: break
		else:
			b += char
			i += 1
	return b.decode( encoding )

## try to load precompiled c-libraries from this directory, if the library is not there try to load from the system.
_clibs_dir = os.path.dirname(os.path.abspath(__file__))

def _load_ctypes_lib( name ):
	if name.startswith('/'):		# if a full path is given bypass all the loading logic and try to load it
		if __os.path.isfile( name ): return ctypes.CDLL( name )
	elif __os.name == 'posix':
		if __sys.platform.startswith('linux'):
			if not name.endswith('.so'): name += '.so'
			if not name.startswith('lib'): name = 'lib' + name

			if IS32BIT: 	path = __os.path.join(_clibs_dir,'linux32')
			else: 		path = __os.path.join(_clibs_dir,'linux64')
			url = __os.path.join( path, name )
			if __os.path.isfile( url ): return ctypes.CDLL(url)
			elif __os.path.isfile( '/usr/local/lib/%s'%name ): return ctypes.CDLL('/usr/local/lib/%s'%name)
			elif __os.path.isfile( '/usr/local/lib64/%s'%name ) and not IS32BIT: return ctypes.CDLL('/usr/local/lib64/%s'%name)
			elif __os.path.isfile( '/usr/lib/%s'%name ): return ctypes.CDLL('/usr/lib/%s'%name)
			elif __os.path.isfile( './%s'%name ): return ctypes.CDLL('./%s'%name)
			
			elif __os.path.isfile( '/usr/lib/%s.0'%name ):	# Fedora style
				return ctypes.CDLL('/usr/lib/%s.0'%name )
			elif __os.path.isfile( '/usr/lib64/%s.0'%name ):	# Fedora style
				return ctypes.CDLL('/usr/lib64/%s.0'%name )

			else:	# fallback
				print('[ falling back to loading from current process ]')
				try: return ctypes.CDLL(name)
				except: return ctypes.CDLL('')

		elif sys.platform == 'darwin':
			name += '.dylib'
			if IS32BIT: 	path = os.path.join(_clibs_dir,'osx32')
			else: 		path = os.path.join(_clibs_dir,'osx64')
			url = os.path.join( path, name )
			if os.path.isfile( url ): return ctypes.CDLL(url)
			else: return ctypes.CDLL(name) #fallback

	elif os.name == 'nt':
		name += '.dll'
		if IS32BIT: 	path = os.path.join(_clibs_dir,'win32')
		else: 		path = os.path.join(_clibs_dir,'win64')
		url = os.path.join( path, name )
		if os.path.isfile( url ): return ctypes.CDLL(url)
		else: return ctypes.CDLL(name) #fallback

RPYTHONIC_WRAPPER_FUNCTIONS = {}
RPYTHONIC_WRAPPER_FUNCTIONS_FAILURES = []
RPYTHONIC_AUTOPREFIX_IGNORE = []

## ctypes does not clearly expose these types ##
PyCFuncPtrType = type(ctypes.CFUNCTYPE(ctypes.c_void_p))
PyCArrayType = type( ctypes.c_int * 2 )
PyCPointerType = type( ctypes.POINTER(ctypes.c_int) )
PyCStructType = type( ctypes.Structure )
CArgObject = type( ctypes.byref(ctypes.c_int()) )

class _rpythonic_meta_(object):
	'''
	Reserved Attributes:
		POINTER
		CSTRUCT
		CAST
	'''
	_rpythonic_ = True		# workaround for now, must have a way to know if object is a meta from another module, isinstance(o,_rpythonic_meta_) will fail in those cases. another workaround could be check sys.modules for other rpythonic modules and fetch _rpythonic_meta_ from there.
	def __init__(self, *args, **kw ):							# cheap trick, abuse **kw, and look for "pointer", "cast"
		if kw and 'pointer' not in kw: raise SyntaxError	# sorry, you can not init with keywords
		elif kw and 'pointer' in kw:
			if 'cast' in kw and kw['cast']:
				self.POINTER = ctypes.cast( kw['pointer'], ctypes.POINTER(self.CSTRUCT) )
			else: self.POINTER = kw['pointer']
		else: self.POINTER = ctypes.pointer( self.CSTRUCT(*args) )
		self.POINTER.pyobject = self	# .pyobject is local to this pointer "object"

	def __getattr__(self,name):
		if hasattr( self.POINTER.contents, name ):
			return getattr( self.POINTER.contents, name )

		else:	# when rpythonic failed to generate good bindings - these lookups should be cached
			for parent in self._rpythonic_parent_classes_:
				if hasattr( parent, name ):
					method = getattr( parent, name )	# should check if it really is an unbound method
					if method in parent._rpythonic_unbound_lookup_:
						func = parent._rpythonic_unbound_lookup_[ method ]
						n = func.name
						if len(func.argnames) > 1:
							argnames = func.argnames[ 1 : ]
							a = ',' + '=None,'.join( argnames ) + '=None'
							b = ','.join( argnames )
						else: a = b = ''
						lamb = eval( 'lambda self %s: %s( self.POINTER, %s )' %(a,n,b) )
						setattr( self.__class__, name, lamb )
						#return lamb	# this would return the unbound lambda, must call getattr again
						return getattr( self, name )
					else:
						# this can happen if self also inherits from the same parent class,
						# assume that by continuing this reaches that shared parent class,
						# and the lambda above is created as normal.
						continue
			## last resort, load from global name space ##
			G = globals()
			if name in G: return lambda *args: G[name](self.POINTER, *args)
			else:
				for prefix in self._autoprefix_:
					n = prefix + name
					if n in G: return lambda *args: G[n](self.POINTER, *args)
				print( 'possible auto-prefixes available', self._autoprefix_ )
				raise AttributeError

	def __call__(self, type=False):
		print('calling object is DEPRECATED - use ob.POINTER or ob.CSTRUCT')
		if type: return self.CSTRUCT
		else: return self.POINTER


def _rpythonic_generate_subclass_( name, struct, functions ):
	head = 'class %s( _rpythonic_meta_ ):' %name
	body = [ 
		'_rpythonic_parent_classes_ = []' ,
		'_rpythonic_unbound_lookup_ = {}' 
	]

	names = [ func.name for func in functions ]

	possibles = {}
	rank = []		# rank by longest name
	if len(names) > 3000: print('too many functions to use this hack')
	else:
		for n1 in names:
			prefix = ''
			for i,char in enumerate(n1):
				prefix += char
				if prefix not in possibles:
					possibles[ prefix ] = 0
					for n2 in names:
						if n2.startswith( prefix ):
							possibles[ prefix ] += 1

					if not rank or len(prefix) > len(rank[-1]) and possibles[prefix] > len(names)/4:
						rank.append( prefix )

	top = []
	while rank:
		best = rank.pop()
		if possibles[best] > len(functions)/2 and best not in names:
			if best.endswith('_set_') or best.endswith('_get_'): best = best[ : -4 ]
			elif best.endswith('Set') or best.endswith('Get'): best = best[ : -3 ]

			rem = []
			for other in rank:
				if best.startswith(other): rem.append( other )
			for r in rem: rank.remove( r )

			if best not in top: top.append( best )

		if len(top) > 3: break

	for n in names:		# find shortest prefixes #
		prefix = ''
		for i,char in enumerate(n):		# cammelCase
			if i==0: prefix += char; continue
			if char.isupper() and len(prefix) >= 2: break
			prefix += char
		if prefix and prefix != n and len(prefix) >= 2:
			hits = 0
			for other in names:
				if other.startswith( prefix ): hits += 1
			if hits >= 2 and prefix not in top:
				top.append( prefix )
				if len(top) >= 6: break

	## setup full names
	for func in functions:
		n = func.name
		if len(func.argnames) > 1:
			argnames = func.argnames[ 1 : ]
			a = ',' + '=None,'.join( argnames ) + '=None'
			b = ','.join( argnames )
		else: a = b = ''

		fhead = 'def %s( self %s ):' %(n,a)
		fbody = ['return %s(self.POINTER, %s)' %(func.name,b)]
		g = fhead + '\n\t\t' + '\n\t\t'.join( fbody )
		body.append( g )
		#body.append( '%s._rpythonic_function_ = %s' %(func.name, func.name) )

	## setup short names ##
	for n in names:
		for prefix in top:
			if n.startswith(prefix) and n[len(prefix):] not in names:
				alt = n[ len(prefix) : ]
				if alt and alt != n and alt not in PYTHON_RESERVED_KEYWORDS and not alt.isdigit() and not alt[0].isdigit():
					body.append( '%s = %s' %(alt,n) )
					names.append( alt )

	gen = head + '\n\t' + '\n\t'.join( body )
	try: exec( gen )
	except:
		print( gen )
		raise SyntaxError

	klass = locals()[name]
	klass.CSTRUCT = struct	# ctypes struct class

	klass._autoprefix_ = top
	for func in functions:
		unbound = getattr( klass, func.name )
		klass._rpythonic_unbound_lookup_[ unbound ] = func
		# klass.longname is klass.shortname = False
		# klass.longname == klass.shortname = True
	return klass

def _rpythonic_convert_structs_to_objects():
	G = globals()
	for klass in _OOAPI_:
		altname = name = klass.__name__
		prefix = ''
		for i,char in enumerate(name):
			if i==0: prefix += char; continue
			if char.isupper(): break
			prefix += char
		if prefix and prefix != name:
			hits = 0
			for other in _OOAPI_:
				if other is not klass:
					if other.__name__.startswith( prefix ): hits += 1
			if hits >= 2:
				altname = name[ len(prefix) : ]

		funcs = _OOAPI_[ klass ]
		newklass = _rpythonic_generate_subclass_( altname, klass, funcs )
		klass._rpythonic_wrapper_class_ = newklass
		G[ name ] = newklass	# replace struct with wrapper
		if altname not in G: G[ altname ] = newklass	# safely define with nicer name
		elif altname != name: # odd cases, maybe a function that returns the object, almost never happens.
			print('WARN - not replacing something with struct wrapper:', G[altname] )


def _rpythonic_setup_return_wrappers():
	R = _rpythonic_function_
	for klass in _OOAPI_:
		if klass in _OOAPI_RETURNS_OBJECT_:
			for f in _OOAPI_RETURNS_OBJECT_[klass]:
				f.object_oriented = True
				if not f.return_wrapper:	# just in case the ctypes footer had already defined it, do not overwrite
					f.return_wrapper = klass._rpythonic_wrapper_class_


###############################################################
##						OLD META FUNC						##
###############################################################
def _rpythonic_function_( name, result=ctypes.c_void_p, args=[]):
	mname = '_metafunc_%s' %name
	exec( 'class %s( _rpythonic_metafunc_ ): pass' %mname )
	k = locals()[mname]
	return k( name, result, args )
_OOAPI_ = {}
_OOAPI_RETURNS_OBJECT_ = {}
class _rpythonic_metafunc_(object):
	def __init__(self, name, result=ctypes.c_void_p, args=[]):
		self.name = name
		self.result = result
		self.argtypes = []		# can dynamically change CFUNCTYPE trick
		self.argnames = []
		self.argtypestypes = []
		for i,arg in enumerate(args):
			n,t = arg
			if n in PYTHON_RESERVED_KEYWORDS: n = 'C_'+n
			if n in self.argnames: n = '%s%s' %(n,i)
			self.argnames.append( n )
			self.argtypes.append( t )
			self.argtypestypes.append( type(t) )		# precomputed for speed

		self.argnames = tuple( self.argnames )				# should never change
		self.numargs = len( self.argtypes )
		self.callbacks = [None] * self.numargs
		self.return_wrapper = None
		self.object_oriented = False
		self.function = None
		for cdll in _CTYPES_CDLLS:	# functions could be multiple libraries
			if hasattr(cdll, self.name):
				func = self.function = getattr(cdll, self.name )
				RPYTHONIC_WRAPPER_FUNCTIONS[ name ] = self
				break
		if not self.function:
			RPYTHONIC_WRAPPER_FUNCTIONS_FAILURES.append( name )

		if self.function: self.reset()

	def change_argument_type( self, name, t ):
		idx = self.argnames.index( name )
		self.argtypes[ idx ] = t
		self.argtypestypes[ idx ] = type(t)
		self.function.argtypes = self.argtypes

	def reset(self):
		if self.argnames:
			a = ',' + '=None,'.join( self.argnames ) + '=None'
			b = ','.join( self.argnames )
		else: a = b = ''

		callmeth = eval( 'lambda self %s: self._call_( %s )' %(a,b) )
		setattr( self.__class__, '__call__', callmeth )

		self.function.restype = self.result
		self.function.argtypes = self.argtypes

		if type( self.result ) is PyCPointerType and type(self.result._type_) is PyCStructType:
			klass = self.result._type_
			if klass not in _OOAPI_RETURNS_OBJECT_: _OOAPI_RETURNS_OBJECT_[klass] = []
			_OOAPI_RETURNS_OBJECT_[klass].append( self )

		self.defaults = []
		for i in range( self.numargs ):
			T = self.argtypes[ i ]
			if type(T) is PyCFuncPtrType:
				p = T()	# func pointers can not be None
				self.defaults.append( p )
				self.callbacks[ i ] = p					# save reference
			elif T in (ctypes.c_int, ctypes.c_uint, ctypes.c_long, ctypes.c_ulong):
				self.defaults.append( 0 )
			elif T in (ctypes.c_float, ctypes.c_double):
				self.defaults.append( .0 )
			else: self.defaults.append( None )	# None is allowed for all other types

			## generate OO API ##
			if i == 0 and type(T) is PyCPointerType and type(T._type_) is PyCStructType:
				klass = T._type_
				if klass not in _OOAPI_: _OOAPI_[ klass ] = []
				_OOAPI_[ klass ].append( self )

	def _call_( self, *args ):			# allow flexible calling types
		cargs = _list( self.defaults )
		for i,arg in enumerate(args):
			if isinstance( arg, _rpythonic_meta_ ): arg = arg.POINTER
			elif hasattr( arg, '_rpythonic_' ): arg = arg.POINTER		# workaround - instance from another module

			t = type(arg)
			k = self.argtypes[ i ]
			kt = self.argtypestypes[ i ]
			if arg is None and cargs[i] is not None:	# use user defaults, very rare cases
				continue

			elif t is bool and k is ctypes.c_int:
				if arg: cargs[i] = 1	#ctypes.c_int(1)
				else: cargs[i] = 0	#ctypes.c_int(0)

			elif t in (list,tuple):				# convert lists and tuples into array
				if kt is PyCArrayType:
					cargs[ i ] = k(*arg)
				elif kt is PyCStructType:
					if k._array_wrapper_: cargs[ i ] = k(arg)	# allow easy array init
					else: cargs[ i ] = k(*arg)							# allow multiple args
				elif kt is PyCPointerType:
					cargs[ i ] = _convert_nested_list_to_pointer( k, arg )
				else: assert 0

			elif isinstance( arg, ctypes._Pointer ) and t is not k and kt is PyCPointerType:
				cargs[ i ] = ctypes.cast( arg, k )		# generic's that need to be cast

			elif kt is PyCStructType and isinstance( arg, ctypes._Pointer ):
				cargs[ i ] = arg.contents	# fixed may25

			elif kt is PyCPointerType and not isinstance( arg, (ctypes._Pointer,CArgObject) ):
				if t in (int,float,bool): ptr = k( k._type_(arg) )
				elif t is str:
					if not _ISPYTHON2: arg = arg.encode('utf-8')	# encode to ascii in python3
					ptr = ctypes.create_string_buffer(arg)				# correct and pypy compatible
				elif t in (PyCStructType, PyCArrayType):
					ptr = ctypes.cast( ctypes.pointer( arg ), k )
				else:
					ptr = arg	# TODO print warning?
				cargs[ i ] = ptr

			elif kt is PyCFuncPtrType:
				if t.__name__ == 'CFunctionType': cargs[ i ] = arg		# assume outside holds pointer
				else:													# this is not safe #
					cargs[ i ] = self.callbacks[ i ] = k( arg )				# assume arg is a callable
			else:
				cargs[ i ] = arg		# directly pass

		## if you define your own return_wrapper, it must take keyword "pointer"
		if self.return_wrapper: return self.return_wrapper( pointer=self.function( *cargs ) )
		else: return self.function( *cargs )



def _convert_nested_list_to_pointer( k, arg ):
	depth = 0; s = k
	while True:
		if type(s) is PyCPointerType:
			s = getattr( s, '_type_' )
			depth += 1
		else: break
	assert depth and depth <= 2
	if depth == 1:
		T = k._type_
		ptr = k( k._type_() )
		for i in range( len(arg) ):
			ptr[ i ] = T( *arg[i] )
	elif depth == 2:
		T = k._type_._type_
		_ptr = k._type_( k._type_._type_() )
		for i in range(len( arg )):
			for j in range( len(arg[i]) ):
				_ptr[ j ] = T( *arg[ i ][ j ] )
		ptr = k( _ptr )
	return ptr


def __freeze_rpythonic_struct( cls, fields ):
	if cls not in _OOAPI_: _OOAPI_[ cls ] = []	# wrap all structs
	try: setattr( cls, '_fields_', fields )
	except:
		print( 'WARN - bad order struct freeze', cls )
		#cls._fields_ = []

class _rpythonic_struct_( ctypes.Structure ):
	_array_wrapper_ = False
	_fields_ = []
	_methods_ = {}
	#def __call__(self): return self
	def __init__(self, *args, **kw ):
		cargs = []
		argtypes = []
		for a in self._fields_: argtypes.append( a[1] )
		if len(args) > len(argtypes): args = [args]	# allow both calling conventions
		for i,arg in enumerate( args ):
			if isinstance( arg, _rpythonic_meta_ ): arg = arg.POINTER

			t = type(arg)
			k = argtypes[ i ]
			if t in (list,tuple):
				if k.__class__.__name__	== 'PyCArrayType':
					cargs.append( k(*arg) )
				elif k.__class__.__name__ == 'PyCStructType':
					if k._array_wrapper_: cargs.append( k(arg) )	# allow easy array init
					else: cargs.append( k(*arg) )							# allow multiple args
			elif isinstance( arg, ctypes._Pointer ) and t is not k:
				cargs[ i ] = ctypes.cast( arg, k )	# generic's that need to be cast
			elif k.__class__.__name__ == 'PyCArrayType' and t in (float,int,bool):
				cargs.append( k(arg) )		# support init array from single value
			else:
				cargs.append( arg )		# directly pass

		ctypes.Structure.__init__(self, *cargs, **kw)


def _rpythonic_make_nice_global_enums_():
	G = globals()
	for name in RPYTHONIC_GLOBAL_ENUMS:
		if '_' in name and name.index('_') <= 4:
			altname = name[ name.index('_') + 1 : ]
			if altname not in G:
				G[altname] = RPYTHONIC_GLOBAL_ENUMS[ name ]

def _rpythonic_clean_up_missing_functions_():
	G = globals()
	for f in RPYTHONIC_WRAPPER_FUNCTIONS_FAILURES:
		G.pop( f )
	print( "C functions loaded: %s" %len(RPYTHONIC_WRAPPER_FUNCTIONS) )
	print( "C functions failed: %s" %len(RPYTHONIC_WRAPPER_FUNCTIONS_FAILURES) )



###### NEW API #########
CTYPES_DLL = None

class _VOID_POINTER_CONTAINER_(object):
	def __init__(self, ptr, name=None):
		self._pointer_ = ptr
		self.name = name
NULL = _VOID_POINTER_CONTAINER_(None,'<null pointer>')


class meta:	# NEW API - allow run time switch from ctypes to rffi
	'''
	Methods:
		RPython will not allow object wrapper around a method (__call__ not allowed)
		keep C function names in list "__cfunctions__"
		rpythonic.enable_rffi( classA, classB )	
		can take advantage of methods in object-method-wrapper,
		generate rffi wrapper and set method on classA, etc.
		replaces object-method-wrapper with rffi-method

	Properties:
		CPython: obj.x=1
		RPython: obj.set_x(1)	
	'''
	METAS = []
	def __init__(self, constructors=[], methods={}, properties={}):
		#global CTYPES_DLL	# TODO update me to using _CTYPES_CDLLS
		#if not CTYPES_DLL:
		#	CTYPES_DLL = _load_ctypes_lib( _clib_name_ )

		self.constructors = constructors
		self.methods = methods
		self.properties = properties
		self.METAS.append( self )



	def __call__(self, cls ):
		print('@meta', cls )

		if not self.constructors:
			lamb = lambda s, _explicit_pointer_=None: setattr(s,'_pointer_',getattr(_explicit_pointer_,'_pointer_')) if hasattr(_explicit_pointer_,'_pointer_') else setattr(s,'_pointer_',_explicit_pointer_)
			lamb._debug = '(no constructor)'
			setattr( cls, '__init__', lamb )
		else:
			con = self._find_best_function( self.constructors )
			cfunc = self._build_cfunc( con )
			setattr( cls, '_%s'%con['name'], cfunc )
			g = self._gen_init( con )
			setattr( cls, '__init__', g )
		
		## set methods ##
		for name in self.methods:
			meth = self.methods[ name ]
			cfuncs = []
			for m in meth['functions']:
				cfunc = self._build_cfunc( m, method=True, static=meth['static'] )
				self._setup_return( cfunc, meth )
				setattr( cls, '_%s'%m['name'], cfunc )
				cfuncs.append( cfunc )

			f = self._find_best_function( meth['functions'] )
			g = self._gen_method( meth, f )
			g._cfuncs = cfuncs
			if meth['static']: g = classmethod( g )
			setattr( cls, name, g )

		for name in self.properties:
			print( 'property:', name )
			p = []
			for f in self.properties[name]:
				cfunc = self._build_cfunc( f )
				setattr( cls, '_%s'%f['name'], cfunc )
				g = self._gen_method( f, f )
				p.append( g )

			setattr( cls, name, property(*p) )

		return cls


	@staticmethod
	def _build_cfunc( info, method=False, static=False ):
		cfunc = getattr(CTYPES_DLL, info['name'])
		if method and not static: argtypes = [ ctypes.c_void_p ]
		else: argtypes = []
		for p in info['parameters']: argtypes.append( eval(p['ctypes_type']) )
		cfunc.argtypes = argtypes
		return cfunc

	@staticmethod
	def _setup_return( cfunc, info ):
		if not info['returns_fundamental']:
			cfunc.restype = ctypes.c_void_p
		elif info['returns_fundamental']:
			cfunc.restype = eval( info['returns_ctypes'] )
		else:
			cfunc.restype = ctypes.c_void_p

	@staticmethod
	def _gen_prepare_args( m ):
		a = []; b = []
		for i,p in enumerate(m['parameters']):
			if 'name' in p: n = p['name']
			else: n = '_unnamed_%s' %i
			if '<' in n: n = '_TODOfixme_%s' %i
			if n in PYTHON_RESERVED_KEYWORDS: n += str(i)
			if p['fundamental']:
				b.append( n )
				s = p['raw_type'].split()
				if 'default' in p:
					d = p['default']
					if p['raw_type'] in ('float', 'double'):
						if d.endswith('f'): d = d[:-1]
						d = d.replace(' ', '.')
						if 'e' in d: d = 0.0
						try: d = float(d)
						except: d = 0.0
					elif ('int' in s or 'size_t' in s) and not d.isdigit(): d = 0
					elif 'char' in s and '"' not in d: d = '""'
					elif d.lower() == 'false': d = False
					elif d.lower() == 'true': d = True
				elif 'char' in s: d = '""'
				elif 'float' in s or 'double' in s: d = 0.0
				elif 'size_t' in s or 'int' in s or 'long' in s or 'short' in s: d = 0
				elif p['raw_type'] == 'bool': d = False
				elif p['raw_type'] in ('void', '(template)'): d = 'NULL'
				else: print( p )
				a.append( n+'=%s' %d )

			else:
				b.append( '%s._pointer_'%n )
				a.append( n+'=NULL' )
		return a, b

	@staticmethod
	def _gen_init( m ):
		a, b = meta._gen_prepare_args( m )
		if a: e = 'lambda _py_self_, %s, _explicit_pointer_=None: ' %(','.join(a))
		else: e = 'lambda _py_self_, _explicit_pointer_=None: '
		e += 'setattr(_py_self_, "_pointer_", _py_self_._%s(%s))' %( m['name'], ','.join(b) )
		e += ' if not _explicit_pointer_ else '
		e += 'setattr(_py_self_, "_pointer_", _explicit_pointer_)'
		print( e )
		lamb = eval( e ); lamb._debug = e; lamb._introspect = m
		return lamb

	@staticmethod
	def _find_best_function( funcs ):
		best = funcs[0]
		score = -1
		if len(funcs) > 1:
			for f in funcs:
				hits = 0
				for p in f['parameters']:
					if p['fundamental']: hits += 1
				if hits and hits == len( f['parameters'] ):
					if hits > score:
						score = hits
						best = f
		return best

	@staticmethod
	def _gen_method( m, f ):
		a, b = meta._gen_prepare_args( f )
		if a: e = 'lambda _py_self_, %s: ' %(','.join(a))
		else: e = 'lambda _py_self_: '

		if 'static' in m and m['static']:	# static in c++ is like a classmethod
			c = '_py_self_._%s( %s )' %( f['name'], ','.join(b) )
		else:
			c = '_py_self_._%s( _py_self_._pointer_, %s )' %( f['name'], ','.join(b) )

		if not m['returns_fundamental']:
			if 'returns_unknown' in m or '<' in m['returns']: c = '_VOID_POINTER_CONTAINER_( %s, name="%s" )' %(c,m['returns'])
			else:
				something = m['returns'].replace('::', '.')
				c = '%s( _explicit_pointer_=%s )' %(something, c)

		e += c; lamb = eval( e )
		lamb._debug = e; lamb._introspect = f
		return lamb


	META_FUNCTIONS = []
	@classmethod
	def function( self, info ):
		print('@meta.function', info['name'] )
		#global CTYPES_DLL
		#if not CTYPES_DLL:
		#	CTYPES_DLL = _load_ctypes_lib( _clib_name_ )

		cfunc = self._build_cfunc( info, method=False, static=True )
		setattr( meta, '_%s'%info['name'], cfunc )
		self._setup_return( cfunc, info )
		a, b = meta._gen_prepare_args( info )
		e = 'lambda %s: ' %(','.join(a))
		c = 'meta._%s( %s )' %( info['name'], ','.join(b) )

		if not info['returns_fundamental']:
			if 'returns_unknown' in info or '<' in info['returns']:
				c = '_VOID_POINTER_CONTAINER_( %s, name="%s" )' %(c,info['returns'])
			else:
				something = info['returns'].replace('::', '.')
				c = '%s( _explicit_pointer_=%s )' %(something, c)
		e += c
		lamb = eval( e )
		lamb._debug = e
		lamb._introspect = info
		return lamb



def _rpythonic_strip_prefixes_( prefixes ):
	G = globals()
	names = _list(G.keys())	# ensure list in py3
	for name in names:
		for prefix in prefixes:
			if name.startswith( prefix ):
				newname = name[ len(prefix) : ]
				if newname and newname not in G:
					G[ newname ] = G[ name ]


########################################################
##				Load Dynamic Libaries					##
def _rpythonic_load_dynamic_libraries(names):
	global _CTYPES_CDLLS
	for name in names:
		cdll = _load_ctypes_lib( name )
		if cdll:
			print('[[dynamic library loaded: %s]]' %name)
			_CTYPES_CDLLS.append( cdll )
#----------------------------------------------------------#
print( os.path.abspath( os.path.curdir ) )

