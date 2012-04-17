# Neo-Rpython - April 18, 2012
# by Brett and The Blender Research Lab.
# License: BSD
import inspect

class Cache(object):
	def __init__( self, translator, debug=True ):
		import pypy.objspace.flow.model

		self.translator = translator
		self.functions = translator.context._prebuilt_graphs	# dict - func:graph
		self.graphs = translator.driver.translator.graphs		# list - graphs
		self.debug = debug
		self.variable_class_cache = {}	# var : class

		## check all simple_call ops, and build new graphs as needed ##
		self.graphs_using_function = {}	# func : [graphs]
		for graph in self.graphs:
			for block in graph.iterblocks():
				for op in block.operations:
					if op.opname != 'simple_call': continue
					print( op )
					if isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
						a = op.args[0].value
						print( 'XXX', a )
						if a in __builtins__.__dict__.values(): print('BUILTIN')
						elif inspect.isfunction( a ):
							if a.func_name not in 'rpython_print_item rpython_print_newline'.split():
								print('USER FUNC')
								if a not in self.functions:
									self.build_graph( a )
									if a not in self.graphs_using_function: self.graphs_using_function[ a ] = []
									assert graph not in self.graphs_using_function[ a ]
									self.graphs_using_function[ a ].append( graph )

		print('@'*80)

	def enter_block(self, block):
		self._block = block
		self._block_method_cache = {}	# (class,func-name) : method_var
		self._block_insert = []

	def leave_block( self, block ):
		assert block is self._block
		## insert the get-method-op before the simple_call ##
		insert = self._block_insert
		modified = bool( insert )
		while insert:
			op, getop = insert.pop()
			index = block.operations.index( op )
			block.operations.insert( index, getop )
		if modified and self.debug:
			print('------- modified ops -------')
			for op in block.operations: print(op)
			print('-'*80)

	def get_method_var( self, cls, func_name, op ):
		if (cls, func_name) in self._block_method_cache: 	## saves a lookup ##
			method_var = self._block_method_cache[ (cls,func_name) ]
		else:
			import pypy.objspace.flow.model
			## create a new variable to hold the pointer to method ##
			instance_var = op.args[0]
			method_var = pypy.objspace.flow.model.Variable()
			func_const = pypy.objspace.flow.model.Constant( func_name )
			## create a new op to get the method and assign to method_var ##
			getop = pypy.objspace.flow.model.SpaceOperation(
				'getattr',					# opname
				[ instance_var, func_const ],	# op args
				method_var				# op result
			)
			## cache this lookup ##
			self._block_method_cache[ (cls,func_name) ] = method_var
			self._block_insert.append( (op,getop) )

		return method_var

	def build_graph( self, func ):
		assert func not in self.functions
		print('>>> building new graph >>>', func)
		graph = self.translator.context.buildflowgraph( func )
		self.functions[ func ] = graph	# also sets -> t.context._prebuilt_graphs
		return graph

	def get_graph_function( self, graph ):
		for func in self.functions:
			if self.functions[func] is graph: return func

	def get_class_helper( self, graph, block, var ):
		'''
		if the creation of the instance is outside of the block this fails.
		TODO traverse ancestor blocks until class is found.
		'''
		import pypy.objspace.flow.model

		if var in self.variable_class_cache: return self.variable_class_cache[ var ]

		#if block.exits:
		#	print( 'SUB BLOCKS', block.exits )
		#	blocks = [link.target for link in block.exits]
		#	for b in blocks:
		#		cls = get_class_helper( b, var )
		#		if cls: return cls
		if var in block.inputargs:		# if var is in the blocks input args, we need to check the other blocks
			print('VAR in block.inputargs')
			for b in graph.iterblocks():
				if b is block: continue
				check = False
				for link in b.exits:
					if link.target is block: check = link; break
				if not check: continue
				prevar = var
				var = link.args[ block.inputargs.index(var) ]

				for op in b.operations:
					## "assert isinstance(a, A)" the hijacked type decl of Rpython,
					## this always appears in an outside block from the one we are checking ##
					if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ) and len(op.args)==3:
						a,b,c = op.args
						if a.value is isinstance and b is var:
							cls = c.value
							self.variable_class_cache[ prevar ] = cls
							return cls
					elif op.result is var:
						if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
							cls = op.args[0].value
							if type(cls) is type:	# how to ensure its a class?
								self.variable_class_cache[ prevar ] = cls
								return cls
			print('*'*80)
			func = self.get_graph_function( graph )
			for g in self.graphs_using_function[ func ]:
				print('          CHECKING',g, func)
				for b in g.iterblocks():
					for op in b.operations:
						if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
							if op.args[0].value is func:
								if op.args[1] in self.variable_class_cache:
									return self.variable_class_cache[ op.args[1] ]

		else:
			for op in block.operations:
				if op.result is var:
					## check if the instance is created inside this block ##
					if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
						cls = op.args[0].value
						if type(cls) is type:	# how to ensure its a class?
							self.variable_class_cache[ var ] = cls
							return cls


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

				if op.opname in ('getitem','setitem') and isinstance( op.args[0], pypy.objspace.flow.model.Variable ):
					instance_var = op.args[0]
					cls = cache.get_class_helper( graph, block, instance_var )
					if not cls:
						print("WARN, can't find class")
						continue

					if op.opname == 'getitem': func_name = '__getitem__'
					else: func_name = '__setitem__'
					assert hasattr(cls, func_name)

					method_var = cache.get_method_var( cls, func_name, op )
					op.opname = 'simple_call'
					op.args = [ method_var ] + op.args[1:]


				elif op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Variable ):
					instance_var = op.args[0]
					cls = cache.get_class_helper( graph, block, instance_var )
					if not cls: continue

					func_name = '__call__'
					assert hasattr(cls, func_name)

					method_var = cache.get_method_var( cls, func_name, op )
					op.args = [ method_var ] + op.args[1:]


				elif op.opname in ('setattr', 'getattr'):
					instance_var = op.args[0]; name_const = op.args[1]
					name = name_const.value	# <class 'pypy.objspace.flow.model.Constant'>
					cls = cache.get_class_helper( graph, block, instance_var )

					if cls and hasattr(cls, name) and type(getattr(cls,name)) is property:
						prop = getattr(cls,name)

						if cls not in class_props: class_props[ cls ] = []
						if name not in class_props[ cls ]: class_props[ cls ].append( name )

						if op.opname == 'setattr':
							func_name = prop.fset.func_name
						elif op.opname == 'getattr':
							func_name = prop.fget.func_name

						method_var = cache.get_method_var( cls, func_name, op )

						## modify op in-place ##
						if op.opname == 'setattr':
							value = op.args[2]
							op.args = [ method_var, value ]
						elif op.opname == 'getattr':
							op.args = [ method_var ]
						## change the op in-place to a simple_call ##
						op.opname = 'simple_call'

			## insert the get-method-op before the simple_call ##
			cache.leave_block( block )



	if delete_class_properties:
		for cls in class_props:	# delete the properties
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

		def __init__(self):
			self.array = [ 100.0 ]

	def func(arg):
		a = A()
		assert isinstance( a, A )
		a.myattr = 'foo'
		s = a.myattr
		a.myattr = s + 'bar'
		a(99)
		a.array.append( 123.4 )
		a[0] = a[0] + a[0]
		other_func( a )		# TODO
		return 1

	def other_func(a):
		a[0] = a[0] * a[0]
		print(a)

	import pypy.translator.interactive
	T = pypy.translator.interactive.Translation( func, standalone=True, inline=False, gc='ref')
	make_rpython_compatible( T, debug=True )

	## before t.annotate is called the flow-graph can be modified to conform to rpython rules ##
	T.annotate()
	T.rtype()

