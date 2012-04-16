# Neo-Rpython - April 15, 2012
# by Brett and The Blender Research Lab.
# License: BSD

def make_rpython_compatible( blocks, delete_class_properties=True, debug=True ):
	'''
	modifies the flowgraph in place to make it strict-Rpython compatible
	'''
	import pypy.objspace.flow.model

	class_props = {}	# return dict of: class : [ prop names ]

	if debug:
		print('=============== make rpython compatible ===============')

	for block in blocks:
		cache = {}
		insert = []
		if debug: print( block )

		for op in block.operations:
			if debug: print( op )

			if op.opname in ('getitem','setitem'):
				if debug: print('---checking getitem/setitem op---')
				cls = type(op.args[0])
				print( op.args[0], cls )
				print( cls.__module__ )
				print( pypy.objspace.flow.model.Variable.__module__ )
				assert cls is pypy.objspace.flow.model.Variable

				assert isinstance( op.args[0], pypy.objspace.flow.model.Variable )

				instance_var = op.args[0]
				cls = get_class_helper( block, instance_var )
				if not cls:
					print("WARN, can't find class")
					continue

				if op.opname == 'getitem': func_name = '__getitem__'
				else: func_name = '__setitem__'
				assert hasattr(cls, func_name)

				if (cls, func_name) in cache: 	## saves a lookup ##
					method_var = cache[ (cls,func_name) ]
				else:
					## create a new variable to hold the pointer to method ##
					method_var = pypy.objspace.flow.model.Variable()
					func_const = pypy.objspace.flow.model.Constant( func_name )
					## create a new op to get the method and assign to method_var ##
					getop = pypy.objspace.flow.model.SpaceOperation(
						'getattr',					# opname
						[ instance_var, func_const ],	# op args
						method_var				# op result
					)
					## cache this lookup ##
					cache[ (cls,func_name) ] = method_var
					insert.append( (op,getop) )

				op.opname = 'simple_call'
				op.args = [ method_var ] + op.args[1:]


			elif op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Variable ):
				instance_var = op.args[0]
				cls = get_class_helper( block, instance_var )
				if not cls: continue

				func_name = '__call__'
				assert hasattr(cls, func_name)

				if (cls, func_name) in cache: 	## saves a lookup ##
					method_var = cache[ (cls,func_name) ]
				else:
					## create a new variable to hold the pointer to method ##
					method_var = pypy.objspace.flow.model.Variable()
					func_const = pypy.objspace.flow.model.Constant( func_name )
					## create a new op to get the method and assign to method_var ##
					getop = pypy.objspace.flow.model.SpaceOperation(
						'getattr',					# opname
						[ instance_var, func_const ],	# op args
						method_var				# op result
					)
					## cache this lookup ##
					cache[ (cls,func_name) ] = method_var
					insert.append( (op,getop) )

				op.args = [ method_var ] + op.args[1:]


			elif op.opname in ('setattr', 'getattr'):
				instance_var = op.args[0]; name_const = op.args[1]
				name = name_const.value	# <class 'pypy.objspace.flow.model.Constant'>
				cls = get_class_helper( block, instance_var )

				if cls and hasattr(cls, name) and type(getattr(cls,name)) is property:
					prop = getattr(cls,name)

					if cls not in class_props: class_props[ cls ] = []
					if name not in class_props[ cls ]: class_props[ cls ].append( name )

					if op.opname == 'setattr':
						func_name = prop.fset.func_name
					elif op.opname == 'getattr':
						func_name = prop.fget.func_name

					if (cls, func_name) in cache:		## saves a lookup ##
						method_var = cache[ (cls,func_name) ]
					else:
						## create a new variable to hold the pointer to get/set-method ##
						method_var = pypy.objspace.flow.model.Variable()
						func_const = pypy.objspace.flow.model.Constant( func_name )
						## create a new op to get the method and assign to method_var ##
						getop = pypy.objspace.flow.model.SpaceOperation(
							'getattr',					# opname
							[ instance_var, func_const ],	# op args
							method_var				# op result
						)
						## cache this lookup ##
						cache[ (cls,func_name) ] = method_var
						insert.append( (op,getop) )

					## modify op in-place ##
					if op.opname == 'setattr':
						value = op.args[2]
						op.args = [ method_var, value ]
					elif op.opname == 'getattr':
						op.args = [ method_var ]
					## change the op in-place to a simple_call ##
					op.opname = 'simple_call'

		## insert the get-method-op before the simple_call ##
		while insert:
			op, getop = insert.pop()
			index = block.operations.index( op )
			block.operations.insert( index, getop )
		if debug:
			print('------- modified ops -------')
			for op in block.operations: print(op)
			print('-'*80)

	if delete_class_properties:
		for cls in class_props:	# delete the properties
			for name in class_props[ cls ]: delattr( cls, name )

	return class_props		# returns class props to be removed before annotation


def get_class_helper( block, var ):
	'''
	if the creation of the instance is outside of the block this fails.
	TODO traverse ancestor blocks until class is found.
	'''
	import pypy.objspace.flow.model

	for op in block.operations:
		if op.result is var:
			if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
				cls = op.args[0].value
				return cls


############################## TESTING ######################################
if __name__ == '__main__':
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
		a.myattr = 'foo'
		s = a.myattr
		a.myattr = s + 'bar'
		a(99)
		a.array.append( 123.4 )
		a[0] = a[0] + a[0]
		return 1



	import pypy.translator.interactive
	T = pypy.translator.interactive.Translation( func, standalone=True, inline=False, gc='ref')

	graphs = T.driver.translator.graphs
	graph = graphs[0]
	blocks = list(graph.iterblocks())
	links = list(graph.iterlinks())

	make_rpython_compatible( blocks, debug=True )

	## before t.annotate is called the flow-graph can be modified to conform to rpython rules ##
	T.annotate()
	T.rtype()

