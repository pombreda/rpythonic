# Neo-Rpython - April 20, 2012
# by Brett and The Blender Research Lab.
# License: BSD
import inspect

def translate( func, inline=True, gc='ref', functions=[], annotate=True, rtype=True, debug=True, make_rpython=True ):
	assert gc in ('ref', 'framework', 'framework+asmgcroot', 'hybrid')
	import pypy.translator.interactive
	t = pypy.translator.interactive.Translation(
		func,
		standalone=True,	# must be True
		inline=inline,
		gc=gc
	)
	'''
	the Translation instance __init__ contains:
		self.driver = driver.TranslationDriver(overrides=DEFAULTS)
		self.config = self.driver.config
		self.entry_point = entry_point
		self.context = TranslationContext(config=self.config)
		...
		self.update_options(argtypes, kwds)
		self.context.buildflowgraph(entry_point)

	'''
	t.driver.secondary_entrypoints = functions
	'''
	setting t.driver.secondary_entrypoints triggers annotator.build_types on t.annotate()
	annotator.build_types builds a flowgraph, and adds it to graphs
	'''
	if functions:
		for func,argtypes in functions:
			graph = t.context.buildflowgraph( func )
			t.context._prebuilt_graphs[ func ] = graph

	print( 'secondary entry points', t.driver.secondary_entrypoints )
	print('-'*80); print('#### PYPY FLOWGRAPH STEP1 COMPLETE ####'); print('-'*80)
	if make_rpython:
		make_rpython_compatible( t, debug=debug )

	if annotate:
		t.annotate()
		print('-'*80); print('#### PYPY ANNONTATION STEP2 COMPLETE ####'); print('-'*80)
	if rtype:
		t.rtype( None )
		print('-'*80); print('#### PYPY RTYPER STEP3 COMPLETE ####'); print('-'*80)

	return t



class Cache(object):
	def cache_instance_class(self, var, cls):
		self.variable_class_cache[ var ] = cls

	def __init__( self, translator, debug=True ):
		import pypy.objspace.flow.model

		self.translator = translator
		self.functions = translator.context._prebuilt_graphs	# dict - func:graph
		self.graphs = translator.driver.translator.graphs		# list - graphs
		self.debug = debug
		self.variable_class_cache = {}	# var : class	[ this could grow huge ]

		## check all simple_call ops, and build new graphs as needed ##
		self.graphs_using_function = {}	# func : [graphs]
		for graph in self.graphs:
			for block in graph.iterblocks():
				for op in block.operations:
					if op.opname != 'simple_call': continue
					if isinstance( op.args[0], pypy.objspace.flow.model.Constant ) and inspect.isfunction(op.args[0].value):
						func = op.args[0].value
						if func.__module__ == '__builtins__' or func.__module__.startswith('pypy.'):
							## do not flow into builtins or pypy internal functions ##
							continue
						if func not in self.functions:
							self.build_graph( func )
							if func not in self.graphs_using_function: self.graphs_using_function[ func ] = []
							assert graph not in self.graphs_using_function[ func ]
							self.graphs_using_function[ func ].append( graph )


	def enter_block(self, block):
		'''
		deal with one block at a time, called at start of loop to make block rpy compatible
		'''
		self._block = block
		self._block_method_cache = {}	# (instance_var, class, func-name) : method_var
		self._block_insert = []

	def leave_block( self, block ):
		'''
		called at end of loop to make block rpy compatible
		'''
		assert block is self._block
		## insert the get-method-op before the simple_call ##
		insert = self._block_insert
		modified = bool( insert )
		while insert:
			op, getop = insert.pop()
			index = block.operations.index( op )
			block.operations.insert( index, getop )
		if modified and self.debug:
			print('------- modified ops:')
			for op in block.operations: print('\t%s' %op)
		if self.debug: print('='*80)


	def change_to_method_call( self, op, cls, func_name, args, return_type=None ):
		if return_type: self.variable_class_cache[ op.result ] = return_type

		instance_var = op.args[0]
		if instance_var not in self.variable_class_cache:
			self.variable_class_cache[ instance_var ] = cls

		method_var = self.get_method_var( instance_var, cls, func_name, op )
		op.opname = 'simple_call'
		op.args = [ method_var ] + args

	def get_method_var( self, instance_var, cls, func_name, op ):
		'''
		if method variable has already been created in this block,
		return it from cache; otherwise create new one a queue insertion.
		'''
		if (instance_var, cls, func_name) in self._block_method_cache: 	## saves a lookup ##
			method_var = self._block_method_cache[ (instance_var,cls,func_name) ]
		else:
			import pypy.objspace.flow.model
			## create a new variable to hold the pointer to method ##
			#instance_var = op.args[0]
			method_var = pypy.objspace.flow.model.Variable()
			func_const = pypy.objspace.flow.model.Constant( func_name )
			## create a new op to get the method and assign to method_var ##
			getop = pypy.objspace.flow.model.SpaceOperation(
				'getattr',					# opname
				[ instance_var, func_const ],	# op args
				method_var				# op result
			)
			## cache this lookup ##
			self._block_method_cache[ (cls,instance_var,func_name) ] = method_var
			self._block_insert.append( (op,getop) )

		return method_var

	def build_graph( self, func ):
		'''
		caches a new flow-graph
		'''
		assert func not in self.functions
		graph = self.translator.context.buildflowgraph( func )
		self.functions[ func ] = graph	# also sets -> t.context._prebuilt_graphs
		return graph

	def get_graph_function( self, graph ):
		'''
		to be safe, not using graph._function_
		'''
		for func in self.functions:
			if self.functions[func] is graph: return func

	def get_class_helper( self, graph, block, var ):
		'''
		if the creation of the instance is outside of the block this fails.
		TODO traverse ancestor blocks until class is found.
		'''
		import pypy.objspace.flow.model

		if var in self.variable_class_cache: return self.variable_class_cache[ var ]

		## first check if the variable is in the block's input args,
		## if so then it could be an "assert isinstance(a,A)"
		## or it could be a simple_call from another graph to this function ##
		if var in block.inputargs:
			#print('VAR in block.inputargs', var)
			for b in graph.iterblocks():
				if b is block: continue
				check = False
				for link in b.exits:
					if link.target is block: check = link; break
				if not check: continue
				v = link.args[ block.inputargs.index(var) ]

				for op in b.operations:
					## "assert isinstance(a, A)" [the hijacked type decl of Rpython],
					## this always appears in an outside block from the one we are checking ##
					if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ) and len(op.args)==3:
						a,b,c = op.args
						if (a.value is isinstance) and b is v:
							cls = c.value
							self.variable_class_cache[ var ] = cls
							return cls
					elif op.result is v:
						if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
							cls = op.args[0].value
							if inspect.isclass(cls):
								self.variable_class_cache[ var ] = cls
								return cls
						else: assert 0

			#print('*'*80)
			assert var in block.inputargs
			if block is not graph.startblock:
				var, link = self.trace_var_to_startblock( var, graph, block )
				#print('TRACED', var, link)

			func = self.get_graph_function( graph )
			if func in self.graphs_using_function:
				for g in self.graphs_using_function[ func ]:
					#print('          CHECKING',g, func)
					for b in g.iterblocks():
						for op in b.operations:
							if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
								if op.args[0].value is func:	# found an operation that calls this graph/function
									iargs = graph.startblock.inputargs
									assert len(iargs) == len(op.args)-1
									index = graph.startblock.inputargs.index( var )
									v = op.args[ index+1 ]
									if v in self.variable_class_cache: return self.variable_class_cache[ v ]
									else: assert 0

		else:
			## check for simple case, the instance is created inside the block we are checking ##
			for op in block.operations:
				if op.result is var:
					if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
						cls = op.args[0].value
						if inspect.isclass(cls):
							self.variable_class_cache[ var ] = cls
							return cls
					elif op.opname == 'getattr' or 1:
						if isinstance( op.args[0], pypy.objspace.flow.model.Variable ):
							v = op.args[0]
							if v in self.variable_class_cache:
								return self.variable_class_cache[ v ]
							else:
								#print('UNKNOWN', v, op)
								pass	# probably a builtin type
						else:
							#assert 0	# TODO easy
							pass
					else:
						print('TODO', op)

	def trace_var_to_startblock( self, var, graph, block ):
		#print('trace', var, graph, block)
		for b in graph.iterblocks():
			if b is block: continue
			check = False
			for link in b.exits:
				if link.target is block: check = link; break
			if not check: continue
			oldvar = var
			var = link.args[ block.inputargs.index(var) ]
			if b is graph.startblock:
				return var, link
			else:
				return self.trace_var_to_startblock( var, graph, b )





#############################################################################
#############################################################################
OVERLOAD_OPS = {
	'add'	:	'+',
	'sub'	:	'-',
	'mul'	:	'*',
	'div'		:	'/',
}


def make_rpython_compatible( translator, delete_class_properties=True, debug=True ):
	'''
	modifies the flowgraph in place to make it strict-Rpython compatible
	'''
	if debug: print('=============== make rpython compatible ===============')
	import pypy.objspace.flow.model

	class_props = {}	# return dict of: class : [ prop names ]
	cache = Cache( translator, debug=debug )

	graphs = translator.driver.translator.graphs
	for graph in graphs:
		blocks = list(graph.iterblocks())
		for block in blocks:
			cache.enter_block( block )
			if debug: print( block )

			for op in block.operations:
				if debug: print( op )

				## check for a simple_call that creates a new instance, if so save it in cache ##
				if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ) and inspect.isclass( op.args[0].value ):
					cache.cache_instance_class( op.result, op.args[0].value )

				##################################################################
				if not isinstance( op.args[0], pypy.objspace.flow.model.Variable ): continue
				instance_var = op.args[0]

				if op.opname.startswith('inplace_') or op.opname in OVERLOAD_OPS:
					cls = cache.get_class_helper( graph, block, instance_var )
					if not cls:
						#print("WARN, can't find class")	# dont warn, probably a builtin.
						continue
					if cls in (bool, int, float, list, dict): continue

					if op.opname.startswith('inplace_'):
						tag = op.opname.split('_')[-1]
						if hasattr(cls, '__i%s__'%tag): func_name = '__i%s__'%tag	# this is expected
						elif hasattr(cls, '__%s__'%tag): func_name = '__%s__'%tag	# funny Python syntax rule
						else: raise SyntaxError
					else:
						tag = op.opname
						if hasattr(cls, '__%s__'%tag): func_name = '__%s__'%tag		# this is expected
						elif hasattr(cls, '__i%s__'%tag): func_name = '__i%s__'%tag	# funny syntax rule
						else: raise SyntaxError

					if op.opname.startswith('inplace_'):
						## need to patch all following op.args that use this op.result,
						## to use the instance_var instead - UGLY! ##
						for otherop in block.operations[ block.operations.index(op) : ]:
							if op.result in otherop.args:
								otherop.args[ otherop.args.index(op.result) ] = instance_var
						#op.result = instance_var	# not a valid flow-graph?

					cache.change_to_method_call( op, cls, func_name, op.args[1:], return_type=cls )



				elif op.opname in ('getitem','setitem'):		# something[ index ]
					cls = cache.get_class_helper( graph, block, instance_var )
					if not cls:
						print("WARN, can't find class")
						continue

					if op.opname == 'getitem': func_name = '__getitem__'
					else: func_name = '__setitem__'
					assert hasattr(cls, func_name)

					cache.change_to_method_call( op, cls, func_name, op.args[1:] )


				elif op.opname == 'simple_call':
					cls = cache.get_class_helper( graph, block, instance_var )
					if not cls: continue

					func_name = '__call__'
					assert hasattr(cls, func_name)

					cache.change_to_method_call( op, cls, func_name, op.args[1:] )


				elif op.opname in ('setattr', 'getattr'):
					assert isinstance(op.args[1], pypy.objspace.flow.model.Constant)
					name_const = op.args[1]; name = name_const.value
					cls = cache.get_class_helper( graph, block, instance_var )

					if cls and hasattr(cls, name) and type(getattr(cls,name)) is property:
						prop = getattr(cls,name)

						if cls not in class_props: class_props[ cls ] = []
						if name not in class_props[ cls ]: class_props[ cls ].append( name )

						if op.opname == 'setattr':
							func_name = prop.fset.func_name
						elif op.opname == 'getattr':
							func_name = prop.fget.func_name

						args = []
						if op.opname == 'setattr': args.append( op.args[2] )
						cache.change_to_method_call( op, cls, func_name, args )


			## insert the get-method-op before the simple_call ##
			cache.leave_block( block )



	if delete_class_properties:
		## this is required to avoid "degenerate to SomeObject" error by the annotation phase
		for cls in class_props:
			for name in class_props[ cls ]: delattr( cls, name )

	return class_props		# returns class props to be removed before annotation




############################## TESTING ######################################
if __name__ == '__main__':
	import sys
	sys.path.append( '../' )

	class A(object):
		def set_myattr(self,v): self.myattr = v
		def get_myattr(self): return self.myattr
		'''
		if myattr is not removed from the class before T.annotate() is called,
		then annotate will fail with a Degenerates to SomeObject error!
		'''
		myattr = property( get_myattr, set_myattr )
		def __call__(self, arg): self.xxx = arg
		def __getitem__(self, index): return self.array[ index ]
		def __setitem__(self, index, value): self.array[ index ] = value
		def __iadd__(self, arg): self.xxx += arg
		def __add__(self,arg):
			a = A()
			a.xxx = self.xxx + arg
			return a
		def __imul__(self,arg): self.xxx *= arg

		def __init__(self):
			self.array = [ 100.0 ]

	def func(arg):
		a = A()
		#assert isinstance( a, A )
		a.myattr = 'foo'
		s = a.myattr
		a.myattr = s + 'bar'
		a(99)
		a.array.append( 123.4 )
		a[0] = a[0] + a[0]
		other_func( a, a[0] )
		a += 420
		a += 421
		b = a + 999
		a *= 10
		print(b)
		b += 25
		c = b + 4000
		print(c)
		return 1

	def other_func(a, b):
		if b:	# makes a new flow-graph block
			if a and a[0]:
				a[0] = a[0] * a[0]
				print(a)
				print(b)



	if '--jit-unroll' in sys.argv:
		import rpyllvmjit
		import pypy.rlib.unroll
		if '--python' in sys.argv:
			CONST_VALUES = range(10)
		else:
			CONST_VALUES = pypy.rlib.unroll.unrolling_iterable( range(10) )
		def unroll_test(a, b):
			c = 0
			for v in CONST_VALUES:
				c += a
				c += b
				c += v
			return c

		if '--python' in sys.argv:
			import time
			start = time.time()
			for i in range(1):
				a = unroll_test( 400, 20 )
			print('end of python benchmark:', time.time()-start)
			assert 0

		T = translate( lambda a: 1, functions=[ (unroll_test,(int,int)) ], debug='--debug' in sys.argv, make_rpython=False )
		jit = rpyllvmjit.JIT( [T.driver.translator.graphs[1]] )
		a = jit.call('unroll_test', 400, 20 )
		print('jit-test:', a)

		## the ops remain unoptimized even after calling T.source_c() - LLVM wins!
		#T.source_c()
		#for block in T.driver.translator.graphs[1].iterblocks():
		#	print('BLOCK',block)
		#	for op in block.operations: print(op)
		if '--benchmark' in sys.argv:
			import time
			start = time.time()
			for i in range(1):
				a = jit.call('unroll_test', 400, 20 )
			print('end of benchmark:', time.time()-start)



	elif '--jit' in sys.argv:
		import rpyllvmjit
		def simple_test(a, b):
			c = 0
			while c < 100000*100000:
				c += a + b
			return c

		T = translate( lambda a: 1, functions=[ (simple_test,(int,int)) ] )
		jit = rpyllvmjit.JIT( [T.driver.translator.graphs[1]], optimize=2 )

		if '--benchmark' in sys.argv:
			import time
			start = time.time()
			a = jit.call('simple_test', 1, 1 )
			print('end of benchmark:', time.time()-start)
			print('test result:', a)
		else:
			a = jit.call('simple_test', 1, 1 )
			print('jit-test:', a)



	else:
		T = translate( func, annotate='--test' in sys.argv, rtype='--test' in sys.argv )


