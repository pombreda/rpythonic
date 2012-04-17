# Neo-Rpython - April 17, 2012
# by Brett and The Blender Research Lab.
# License: BSD

def make_rpython_compatible( graph, delete_class_properties=True, debug=True ):
	'''
	modifies the flowgraph in place to make it strict-Rpython compatible
	'''
	if debug: print('=============== make rpython compatible ===============')
	import pypy.objspace.flow.model

	#class_props = process_blocks( graph.iterblocks() )
	#blocks = [ link.target for link in graph.iterlinks() ]
	#process_blocks( blocks )

	class_props = {}	# return dict of: class : [ prop names ]
	blocks = list(graph.iterblocks())
	for block in blocks:
		cache = {}
		insert = []
		if debug: print( block )

		#if block.exits:
		#	print( 'SUB BLOCKS', block.exits )
		#	process_blocks( [link.target for link in block.exits] )

		for op in block.operations:
			if debug: print( op )

			if op.opname in ('getitem','setitem') and isinstance( op.args[0], pypy.objspace.flow.model.Variable ):
				instance_var = op.args[0]
				cls = get_class_helper( graph, block, instance_var )
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
				cls = get_class_helper( graph, block, instance_var )
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
				cls = get_class_helper( graph, block, instance_var )

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
		modified = bool( insert )
		while insert:
			op, getop = insert.pop()
			index = block.operations.index( op )
			block.operations.insert( index, getop )
		if modified and debug:
			print('------- modified ops -------')
			for op in block.operations: print(op)
			print('-'*80)



	if delete_class_properties:
		for cls in class_props:	# delete the properties
			for name in class_props[ cls ]: delattr( cls, name )

	return class_props		# returns class props to be removed before annotation


def get_class_helper( graph, block, var ):
	'''
	if the creation of the instance is outside of the block this fails.
	TODO traverse ancestor blocks until class is found.
	'''
	import pypy.objspace.flow.model

	#if block.exits:
	#	print( 'SUB BLOCKS', block.exits )
	#	blocks = [link.target for link in block.exits]
	#	for b in blocks:
	#		cls = get_class_helper( b, var )
	#		if cls: return cls
	if var in block.inputargs:		# if var is in the blocks input args, we need to check the other blocks
		for b in graph.iterblocks():
			if b is block: continue
			check = False
			for link in b.exits:
				if link.target is block: check = link; break
			if not check: continue
			var = link.args[ block.inputargs.index(var) ]

			for op in b.operations:
				## "assert isinstance(a, A)" the hijacked type decl of Rpython,
				## this always appears in an outside block from the one we are checking ##
				if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ) and len(op.args)==3:
					a,b,c = op.args
					if a.value is isinstance and b is var:
						return c.value
				elif op.result is var:
					if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
						cls = op.args[0].value
						if type(cls) is type:	# how to ensure its a class?
							return cls

	else:
		for op in block.operations:
			if op.result is var:
				## check if the instance is created inside this block ##
				if op.opname == 'simple_call' and isinstance( op.args[0], pypy.objspace.flow.model.Constant ):
					cls = op.args[0].value
					if type(cls) is type:	# how to ensure its a class?
						return cls


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
		#other_func( a )		# TODO
		return 1

	def other_func(a):
		#a[0] = a[0] * a[0]
		print(a)

	import pypy.translator.interactive
	T = pypy.translator.interactive.Translation( func, standalone=True, inline=False, gc='ref')

	graphs = T.driver.translator.graphs
	make_rpython_compatible( graphs[0], debug=True )

	## before t.annotate is called the flow-graph can be modified to conform to rpython rules ##
	#T.annotate()
	#T.rtype()

