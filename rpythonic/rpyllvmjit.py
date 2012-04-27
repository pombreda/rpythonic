# Rpython LLVM JIT - April 20, 2012
# by Brett and The Blender Research Lab.
# License: BSD
import llvm.core
import llvm.ee
import llvm.passes
import ctypes
import sys

DEBUG = '--debug' in sys.argv

######################################################################

class Block(object):
	def show(self):
		import inspect
		for name in dir(self):
			if name.startswith('__'): continue
			attr = getattr(self,name)
			if not inspect.ismethod(attr):
				print(name)
				if name=='link' and attr: print('\t%s\targs:%s' %(attr,attr.args))
				else: print('\t%s' %attr)
		print('_'*80)
		for child in self.children: child.show()


	def __init__(self, block, link=None, parent=None, graph=None):
		self.llvm_block = None

		self.block = block
		self.input = block.inputargs
		self.link = link
		self.parent = parent
		self.graph = graph

		self.exit_vars = exit_vars = []
		for link in block.exits:
			for arg in link.args:
				if arg not in exit_vars: exit_vars.append( arg )

		self.mutates_from_to = mutates_from_to = {}
		self.mutates_to_from = mutates_to_from = {}
		self.read_only = read_only = []
		self.read_pass = read_pass = []
		print('----------> %s	%s' %(block,block.inputargs))
		for op in block.operations:
			print(op)
			for arg in op.args:
				if arg in block.inputargs:
					if arg in exit_vars:
						if arg not in read_pass: read_pass.append( arg )
					else:
						chain = []
						self.trace_result_to_escape( op.result, chain )
						if chain and chain[-1] in exit_vars:
							mutates_from_to[ arg ] = chain[-1]
							mutates_to_from[ chain[-1] ] = arg	# TO : FROM
							## later when building llvm instructions - check if op.result is in mutates_to_from ##
						else:
							if arg not in read_only: read_only.append( arg )

		## for exit blocks to quickly check their input and get the exit var ##
		import pypy.objspace.flow.model
		self.exit_lookup_var = {}	## block input arg : link output arg
		self.exit_lookup_traced = {}
		self.escape_vars = {}		## escape-result : [ link.target.inputargs[...] ]
		for var in exit_vars:
			print('checking exit var', var)
			if var in mutates_to_from or var in read_pass:
				for link in self.block.exits:
					if var in link.args:
						target_var = link.target.inputargs[ link.args.index(var) ]
						self.exit_lookup_var[ target_var ] = var
						if var in mutates_to_from:
							self.exit_lookup_traced[ target_var ] = mutates_to_from[ var ]
						else:
							print('----read-pass', var)
						if var not in self.escape_vars:
							self.escape_vars[ var ] = []
						self.escape_vars[ var ].append( target_var )

			else:
				if isinstance( var, pypy.objspace.flow.model.Constant ):
					const = var
					for link in self.block.exits:
						if const in link.args:
							target_var = link.target.inputargs[ link.args.index(const) ]
							self.exit_lookup_var[ target_var ] = const
				else:
					#print('--------warn---------',var)
					for link in self.block.exits:
						if var in link.args:
							target_var = link.target.inputargs[ link.args.index(var) ]
							if var not in self.escape_vars:
								self.escape_vars[ var ] = []
							self.escape_vars[ var ].append( target_var )


		self.children = []
		self.is_loop = False
		for link in self.block.exits:
			if link.target is self.block:
				assert not self.is_loop
				self.is_loop = link
			else:
				B = Block( link.target, link=link, parent=self, graph=self.graph )
				self.children.append( B )


	def trace_result_to_escape(self, result, chain ):
		for op in self.block.operations:
			for arg in op.args:
				if arg is result:
					assert op.result not in chain	# is this correct?
					chain.append( op.result )
					if op.result in self.exit_vars: return
					else: self.trace_result_to_escape( op.result, chain )


	def trace_variable_flow( self, block ):
		'''
		This is called when the entry block is first created, it flattens the variable flow.
		PASS_PROMOTE_MEMORY_TO_REGISTER requires all allocations happen in the entry block.

		allocations = []	all entry-allocations	(mutable vars)

		first check all op.args if any arg is in block.inputargs,
		then trace flow of results to the end of block, and check if it exits the block,
		if result exits then entry-allocation needs to store that result:
			(we can be sure it needs to be stored if the block loops and exit var is in inputs)

		results = {}
			[ op.result ] = entry-allocation
			flat list of all operation results that change the value of a entry-allocation,
			this is harder to know.  The op.result must be checked if it exits the block,
			results that exit must be allocations

example:
				operation "int_add" - if the 

		opargs = {}
			[ arg ] = entry-allocation
			flat list of all operation arguments that trace back to an entry-allocation

		'''
		pass

	def setup_llvm_translation( self, llvm_func ):
		'''
		called once at start, only on root block.
		'''
		assert self.block is self.graph.startblock
		Block.llvm_func = llvm_func
		self.llvm_block = llvm_func.append_basic_block( 'entry' )
		Block.llvm_builder = llvm.core.Builder.new( self.llvm_block )
		Block.var_cache = vmap = {}
		Block.ALLOCATIONS = {}
		Block.INSTANCE_VARS = {}

		graph = self.graph
		for i,pypyarg in enumerate(graph.startblock.inputargs):	# assume Variables
			var = llvm_func.args[ i ]
			var.name = pypyarg.name
			vmap[ pypyarg ] = var
			for link in graph.startblock.exits:
				if pypyarg in link.args:
					arg2 = link.target.inputargs[ link.args.index(pypyarg) ]
					vmap[ arg2 ] = var



	def get_llvm_block(self):
		if not self.llvm_block:
			if self.block is self.graph.returnblock:
				self.llvm_block = self.llvm_func.append_basic_block( 'return' )
			else:
				self.llvm_block = self.llvm_func.append_basic_block( 'block' )
		return self.llvm_block

	def llvm_type( self, var ):
		import pypy.rpython.lltypesystem.lltype
		#if isinstance(ret, pypy.rpython.lltypesystem.lltype.Number):
		#	llret = llvm.core.Type.int(32)
		ctype = var.concretetype
		#print(ctype, type(ctype))
		if isinstance(ctype, pypy.rpython.lltypesystem.lltype.Ptr):	# TODO
			#return llvm.core.Type.pointer( llvm.core.Type.void() )
			return None
		else:
			return self.types[ ctype._name ]


	def allocate( self, arg ):
		if not self.parent:		# only allocate in entry
			if arg in self.ALLOCATIONS: return self.ALLOCATIONS[ arg ]
			else:
				print('NEW ALLOCA',arg)
				self.llvm_builder.position_at_beginning( self.get_llvm_block() )
				stackvar = self.llvm_builder.alloca( self.llvm_type(arg), 'st_'+arg.name )

				a = self.lload(
					self.exit_lookup_var[ arg ],
					allocate=False,	# do not allow allocation!
				)
				self.llvm_builder.store( a, stackvar )

				self.ALLOCATIONS[ arg ] = stackvar
				return stackvar
		else:
			if arg in self.parent.exit_lookup_traced:
				alias = self.parent.exit_lookup_traced[ arg ]
				if alias in self.ALLOCATIONS:
					st = self.ALLOCATIONS[ alias ]
				else:
					st = self.parent.allocate( alias )				
			else:
				st = self.parent.allocate( arg )

			for esc_var in self.escape_vars:		## if the result of some operation requires a store, fast cache it here ##
				if arg in self.escape_vars[ esc_var ]:
					self.ALLOCATIONS[ esc_var ] = st
					for other in self.escape_vars[ esc_var ]:
						self.ALLOCATIONS[ other ] = st

			self.ALLOCATIONS[ arg ] = st
			return st


	def lload( self, arg, force_type=None, allocate=True ):
		import pypy.objspace.flow.model

		if arg in self.var_cache:
			return self.var_cache[ arg ]
		elif arg in self.ALLOCATIONS:		## TODO this is a problem... the calling end needs to know to load it
			return self.ALLOCATIONS[ arg ]

		elif arg in self.input and allocate:
			if arg in self.ALLOCATIONS: a = self.ALLOCATIONS[ arg ]
			else:
				a = self.allocate( arg )
				self.ALLOCATIONS[ arg ] = a
				self.llvm_builder.position_at_end( self.get_llvm_block() )
			return self.llvm_builder.load( a, arg.name )

		elif isinstance( arg, pypy.objspace.flow.model.Constant ):
			if type(arg.value) is float:
				if force_type: return llvm.core.Constant.real( force_type, arg.value )
				else: return llvm.core.Constant.real( self.types['Float'], arg.value )
			else:
				if force_type: return llvm.core.Constant.int( force_type, arg.value )
				else: return llvm.core.Constant.int( self.types['Signed'], arg.value )
		else:
			assert 0

	def make_vector(self, *args, **kw):
		builder = self.llvm_builder

		t = self.types['Signed']
		length = len(args)
		if 'length' in kw: length = kw['length']
		if 'type' in kw:
			if kw['type'] in (float,'float32'):
				t = llvm.core.Type.float()
				assert length==4
			elif kw['type'] in ('float64', 'double'):
				t = llvm.core.Type.double()
				assert length==2
			elif kw['type'] in (int,'int32'):
				t = llvm.core.Type.int(32)
				assert length==4
			elif kw['type'] == 'int64':
				t = llvm.core.Type.int(64)
				assert length==2
			else:
				raise NotImplemented

		vtype = llvm.core.Type.vector( t, length )
		stackvar = builder.alloca( vtype, 'stack_vec' )

		if type(args[0]) in (int,float):
			vargs = []
			for arg in args:
				if type(arg) is int:
					vargs.append( llvm.core.Constant.int( t, arg ) )
				else:
					vargs.append( llvm.core.Constant.real( t, arg ) )
		else:
			vargs = args		# must be constants?

		if all( [isinstance(a,llvm.core.Constant) for a in vargs] ):
			const = llvm.core.Constant.vector( vargs )
			builder.store( const, stackvar )
			var = builder.load( stackvar, 'vec' )
		else:
			if kw['type'].startswith('float'):
				const_args = [ llvm.core.Constant.real(t,0.0) for i in range(length) ]
			if kw['type'].startswith('int'):
				const_args = [ llvm.core.Constant.int(t,0) for i in range(length) ]

			const = llvm.core.Constant.vector( const_args )
			builder.store( const, stackvar )
			var = builder.load( stackvar, 'vec' )

			for index,carg in enumerate(vargs):
				var = builder.insert_element(
					var,
					carg,
					llvm.core.Constant.int( self.types['int32'], index )
				)
		return var

	def flow_cache_helper2( self, var, op, block ):
		self.var_cache[ op.result ] = var
		others = []
		for link in block.exits:
			if op.result in link.args:
				arg = link.target.inputargs[ link.args.index(op.result) ]
				self.var_cache[ arg ] = var
				others.append( arg )
		return others


	def generate_llvm_ir( self ):
		builder = self.llvm_builder
		builder.position_at_end( self.get_llvm_block() )
		block = self.block
		print(block)
		_make_vecs = []

		for op in block.operations:
			print( op )

			if op.opname == 'same_as':	# can fold?
				print('INSTVARS', self.INSTANCE_VARS)
				if op.args[0] in self.INSTANCE_VARS:
					a = self.INSTANCE_VARS[ op.args[0] ]
				else:
					a = self.lload(op.args[0], allocate=False )
				self.var_cache[ op.result ] = a
				if op.args[0] in self.INSTANCE_VARS:
					self.INSTANCE_VARS[ op.result ] = a


			elif op.opname == 'malloc':		# TODO support structs, for now assume Vector
				gcstruct = op.args[0].value
				name = gcstruct._name
				assert name in self.graph._llvm_hints['vector-classes']
				_make_vecs.append(
					(op, self.graph._llvm_hints['vector-classes'][name])
				)


			elif op.opname == 'direct_call':
				fn = op.args[0].value
				#print(fn, dir(fn))
				#print(fn._obj, type(fn._obj), dir(fn._obj))
				#assert isinstance(fn._obj, pypy.rpython.lltypesystem.lltype._func)

				_func = fn._obj._callable
				fname = _func.func_name
				if fname == '__init__':	# same_as op will come after this
					gcop, vclass = _make_vecs.pop()
					assert not _make_vecs		# GcStruct's are created one by one?
					args = [ self.lload(op.args[2]), self.lload(op.args[3]), self.lload(op.args[4]) ]
					var = self.make_vector( *args, **vclass._llvm_hints )
					self.var_cache[ gcop.result ] = var

					self.INSTANCE_VARS[ gcop.result  ] = var
					if gcop.result in self.escape_vars:
						for other in self.escape_vars[ gcop.result ]:
							self.INSTANCE_VARS[ other  ] = var


					#others = self.flow_cache_helper( var, gcop, block )
					#for other in others:
					#	self._instance_vars[ other ] = var

				elif fname == '__getitem__':
					a = self.INSTANCE_VARS[ op.args[1] ]
					b = self.lload(
						op.args[2],
						force_type=self.types['int32']
					)
					var = builder.extract_element( a,b, 'element' )
					self.var_cache[ op.result ] = var
					#if op.result in self.allocas:
					#	self.builder.store( var, self.allocas[op.result] )


				elif fname == '__setitem__':
					inst = self.INSTANCE_VARS[ op.args[1] ]
					idx = self.lload(op.args[2], force_type=self.types['int32'])
					val = self.lload(op.args[3])
					var = builder.insert_element( inst, val, idx )

					self.var_cache[ op.result ] = var
					#others = self.flow_cache_helper( var, op, block )
					self.INSTANCE_VARS[ op.result  ] = var
					#for other in others: self._instance_vars[ other ] = var


				elif fname == '__add__':
					a = self.INSTANCE_VARS[ op.args[1] ]
					b = self.INSTANCE_VARS[ op.args[2] ]
					var = builder.fadd( a, b, 'vecadd' )
					self.var_cache[ op.result ] = var

					#others = self.flow_cache_helper( var, op, block )
					self.INSTANCE_VARS[ op.result  ] = var
					#for other in others: self._instance_vars[ other ] = var
					if op.result in self.escape_vars:
						for other in self.escape_vars[ op.result ]:
							self.INSTANCE_VARS[ other  ] = var


				else:
					assert 0



			elif op.opname == 'int_lt':	# 1 < 2
				a = self.lload(op.args[0])
				b = self.lload(op.args[1])
				var = builder.icmp( llvm.core.ICMP_ULT, a, b, op.opname )
				var.name = op.result.name
				self.var_cache[ op.result ] = var


			elif op.opname in ('int_add', 'float_add'):
				a = self.lload( op.args[0] )
				b = self.lload( op.args[1] )

				if op.opname == 'int_add':
					var = builder.add( a, b, op.opname )
				elif op.opname == 'float_add':
					var = builder.fadd( a, b, op.opname )

				var.name = op.result.name
				#if op.result in self.allocas:
				#	self.builder.store( var, self.allocas[op.result] )
				self.var_cache[ op.result ] = var

				if op.result in self.escape_vars:
					for other in self.escape_vars[ op.result ]:
						self.var_cache[ other  ] = var


				if op.result in self.exit_vars:
					if op.result in self.mutates_to_from:
						v_from = self.mutates_to_from[ op.result ]
						if v_from in self.INSTANCE_VARS:
							st = self.INSTANCE_VARS[ v_from ]
						else:
							st = self.ALLOCATIONS[ v_from ]
						builder.store( var, st )
					else:	# should have already been cached #
						print(self.ALLOCATIONS)
						st = self.ALLOCATIONS[ op.result ]
						builder.store( var, st )



		if self.is_loop:
			assert len(self.children)==1
			self.llvm_block.name = 'while_loop'
			con_bool = self.lload( self.block.exitswitch )
			link = self.is_loop
			elseblock = self.children[0].get_llvm_block()
			elseblock.name = 'else'
			thenblock = self.llvm_block
			builder.position_at_end( thenblock )
			builder.cbranch( con_bool, thenblock, elseblock )

		if len(self.children)==1 and not self.is_loop:
			child = self.children[0]
			toblock = child.get_llvm_block()
			builder.position_at_end( self.llvm_block )
			builder.branch( toblock )
			child.generate_llvm_ir()
		elif self.children:
			for child in self.children:
				child.generate_llvm_ir()

		elif not self.children:	# return block
			assert self.block is self.graph.returnblock
			assert len(self.input)==1
			#ret = self.lload( self.input[0] )
			print('RETURN BLOCK', self.input[0] )

			st = None
			if self.input[0] in self.parent.exit_lookup_traced:
				a = self.parent.exit_lookup_traced[ self.input[0] ]
				print('EXIT LOOKUP', a)
				if a in self.INSTANCE_VARS: ret = self.INSTANCE_VARS[a]
				else: st = self.allocate( a )
			else:
				st = self.allocate( self.input[0] )

			if st:
				ret = builder.load( st )

			builder.ret( ret )
		else:
			assert 0

######################################################################
class JIT(object):
	'''
	With LLVM IR we don't have to worry about things like this:
		Linux32 and Win32:
			int=32, long=32, long-long=64, void*=32
		Linux64:
			int=32, long=64, long-long=64, void*=64
		Win64:
			int=32, long=32, long-long=64, void*=64

	'''
	types = {
		'int32'	: llvm.core.Type.int(32),	# vector insert_element requires int32

		'Bool'	: llvm.core.Type.int(1),
		'Void'	: llvm.core.Type.void(),
		'Signed'	: llvm.core.Type.int(32),
		'Float'	: llvm.core.Type.float(),	# 32bits
		'Double'	: llvm.core.Type.double(),	# 64bits
	}
	types['Signed']._type = 'int32'
	types['Float']._type = 'float32'
	types['Double']._type = 'float64'

	def llvm_type( self, var ):
		import pypy.rpython.lltypesystem.lltype
		#if isinstance(ret, pypy.rpython.lltypesystem.lltype.Number):
		#	llret = llvm.core.Type.int(32)
		ctype = var.concretetype
		#print(ctype, type(ctype))
		if isinstance(ctype, pypy.rpython.lltypesystem.lltype.Ptr):	# TODO
			#return llvm.core.Type.pointer( llvm.core.Type.void() )
			return None
		else:
			return self.types[ ctype._name ]

	def get_var_or_const( self, pypyarg, force_type=None ):
		import pypy.objspace.flow.model
		if isinstance( pypyarg, pypy.objspace.flow.model.Variable ):
			if pypyarg in self.var_cache:	# assume variable
				v = self.var_cache[ pypyarg ]
				#v.name = pypyarg.name	# ensure nice name
				return v
			else:
				v = self.find_cached_var( pypyarg )
		else:			# assume constant
			if type(pypyarg.value) is float:
				if force_type: return llvm.core.Constant.real( force_type, pypyarg.value )
				else: return llvm.core.Constant.real( self.types['Float'], pypyarg.value )
			else:
				if force_type: return llvm.core.Constant.int( force_type, pypyarg.value )
				else: return llvm.core.Constant.int( self.types['Signed'], pypyarg.value )

	def find_cached_var(self, var):
		print('ERROR:', var)
		assert 0

	def make_vector(self, *args, **kw):
		t = self.types['Signed']
		length = len(args)
		if 'length' in kw: length = kw['length']
		if 'type' in kw:
			if kw['type'] in (float,'float32'):
				t = llvm.core.Type.float()
				assert length==4
			elif kw['type'] in ('float64', 'double'):
				t = llvm.core.Type.double()
				assert length==2
			elif kw['type'] in (int,'int32'):
				t = llvm.core.Type.int(32)
				assert length==4
			elif kw['type'] == 'int64':
				t = llvm.core.Type.int(64)
				assert length==2
			else:
				raise NotImplemented

		vtype = llvm.core.Type.vector( t, length )
		stackvar = self.builder.alloca( vtype, 'stack_vec' )

		if type(args[0]) in (int,float):
			vargs = []
			for arg in args:
				if type(arg) is int:
					vargs.append( llvm.core.Constant.int( t, arg ) )
				else:
					vargs.append( llvm.core.Constant.real( t, arg ) )
		else:
			vargs = args		# must be constants?

		if all( [isinstance(a,llvm.core.Constant) for a in vargs] ):
			const = llvm.core.Constant.vector( vargs )
			self.builder.store( const, stackvar )
			var = self.builder.load( stackvar, 'vec' )
		else:
			if kw['type'].startswith('float'):
				const_args = [ llvm.core.Constant.real(t,0.0) for i in range(length) ]
			if kw['type'].startswith('int'):
				const_args = [ llvm.core.Constant.int(t,0) for i in range(length) ]

			const = llvm.core.Constant.vector( const_args )
			self.builder.store( const, stackvar )
			var = self.builder.load( stackvar, 'vec' )

			for index,carg in enumerate(vargs):
				var = self.builder.insert_element(
					var,
					carg,
					llvm.core.Constant.int( self.types['int32'], index )
				)
		return var

	def flow_cache_helper( self, var, op, block ):
		self.var_cache[ op.result ] = var
		others = []
		for link in block.exits:
			if op.result in link.args:
				arg = link.target.inputargs[ link.args.index(op.result) ]
				self.var_cache[ arg ] = var
				others.append( arg )
		return others




	def flow( self, graph, block=None, indent=0 ):
		import pypy.objspace.flow.model

		func = self.functions[ graph.name ]
		if not block:
			assert not self.blocks and not self.builder
			block = graph.startblock
			blk = func.append_basic_block( 'entry' )
			self.builder = llvm.core.Builder.new( blk )
			self.blocks[ block ] = blk
			self.allocas = {}


			for link in graph.iterlinks():
				assert link.target is not graph.startblock

				for arg in link.args:	# arg is: Variable or Constant
					#if link.prevblock is graph.startblock:
					#	if arg in graph.startblock.inputargs: continue

					escapes = []
					tvar = link.target.inputargs[link.args.index(arg)]
					for sublink in link.target.exits:
						if tvar in sublink.args:
							esc = sublink.target.inputargs[ sublink.args.index(tvar) ]
							escapes.append( esc )
							break
					#if not escapes: continue

					if arg not in self.allocas:
						if isinstance( arg, pypy.objspace.flow.model.Variable ):
							t = self.llvm_type(arg)
							if not t: continue

							stackvar = self.builder.alloca( t, 'st_'+arg.name )
							if link.prevblock is graph.startblock and len(func.args)==len(link.args):	 # TODO FIXME
								print('BROOOOOOOOKEN')
								self.builder.store( func.args[link.args.index(arg)], stackvar )

						else:	# Constant
							stackvar = self.builder.alloca( self.llvm_type(arg), 'st' )
							if type(arg.value) is int:
								const = llvm.core.Constant.int( self.types['Signed'], arg.value )
							elif type(arg.value) is float:
								const = llvm.core.Constant.real( self.types['Float'], arg.value )
							else: raise NotImplemented

							self.builder.store( const, stackvar )
						self.allocas[ arg ] = stackvar

						stackvar = self.allocas[ arg ]
						self.allocas[ tvar ] = stackvar
						for esc in escapes:
							self.allocas[ esc ] = stackvar


		elif block not in self.blocks:	# could be created below
			if block is graph.returnblock:
				blk = func.append_basic_block( 'return' )
			else:
				blk = func.append_basic_block( 'block' )
			self.blocks[ block ] = blk

		blk = self.blocks[ block ]
		self.builder.position_at_end( blk )

		tabs = '\t'*indent
		print(tabs+'_'*40)
		print(tabs+repr(block)+'	inputargs: '+str(block.inputargs))

		get = self.get_var_or_const
		_make_vecs = []

		for op in block.operations:
			print( ('\t'*indent)+str(op) )
			if op.opname == 'malloc':		# TODO support structs, for now assume Vector
				gcstruct = op.args[0].value
				name = gcstruct._name
				assert name in graph._llvm_hints['vector-classes']
				_make_vecs.append( (op, graph._llvm_hints['vector-classes'][name]) )

			elif op.opname == 'same_as':	# can fold?
				self.var_cache[ op.result ] = get(op.args[0])
				if op.args[0] in self._instance_vars:
					self._instance_vars[ op.result ] = get(op.args[0])


			elif op.opname == 'direct_call':
				fn = op.args[0].value
				#print(fn, dir(fn))
				#print(fn._obj, type(fn._obj), dir(fn._obj))
				#assert isinstance(fn._obj, pypy.rpython.lltypesystem.lltype._func)

				_func = fn._obj._callable
				fname = _func.func_name
				if fname == '__init__':	# same_as op will come after this
					gcop, vclass = _make_vecs.pop()
					assert not _make_vecs		# GcStruct's are created one by one?
					args = [ get(op.args[2]), get(op.args[3]), get(op.args[4]) ]
					var = self.make_vector( *args, **vclass._llvm_hints )
					self.var_cache[ gcop.result ] = var
					self._instance_vars[ gcop.result  ] = var
					others = self.flow_cache_helper( var, gcop, block )
					for other in others:
						self._instance_vars[ other ] = var

				elif fname == '__getitem__':
					a = self._instance_vars[op.args[1]]
					b = get(op.args[2], force_type=self.types['int32'])
					var = self.builder.extract_element( a,b, 'element' )
					self.var_cache[ op.result ] = var
					if op.result in self.allocas:
						self.builder.store( var, self.allocas[op.result] )

				elif fname == '__setitem__':
					inst = self._instance_vars[op.args[1]]
					idx = get(op.args[2], force_type=self.types['int32'])
					val = get(op.args[3])
					var = self.builder.insert_element( inst, val, idx )

					others = self.flow_cache_helper( var, op, block )
					self._instance_vars[ op.result  ] = var
					for other in others: self._instance_vars[ other ] = var


				elif fname == '__add__':
					a = self._instance_vars[op.args[1]]
					b = self._instance_vars[op.args[2]]
					var = self.builder.fadd( a, b, 'vecadd' )
					self.var_cache[ op.result ] = var

					others = self.flow_cache_helper( var, op, block )
					self._instance_vars[ op.result  ] = var
					for other in others: self._instance_vars[ other ] = var


				else:
					assert 0

			elif op.opname in ('int_add', 'float_add'):
				if op.args[0] in block.inputargs and op.args[0] in self.allocas:
					a = self.builder.load( self.allocas[op.args[0]], op.args[0].name )
				else:
					a = get(op.args[0])

				if op.args[1] in block.inputargs:
					b = self.builder.load( self.allocas[op.args[1]], op.args[1].name )
				else:
					b = get(op.args[1])

				if op.opname == 'int_add':
					var = self.builder.add( a, b, op.opname )
				elif op.opname == 'float_add':
					var = self.builder.fadd( a, b, op.opname )

				var.name = op.result.name
				if op.result in self.allocas:
					self.builder.store( var, self.allocas[op.result] )
					#var = self.allocas[op.result]	# ??? not correct

				self.var_cache[ op.result ] = var
				for link in block.exits:
					if op.result in link.args:
						arg = link.target.inputargs[ link.args.index(op.result) ]
						self.var_cache[ arg ] = var



			elif op.opname == 'int_lt':	# 1 < 2
				a = get(op.args[0])
				b = get(op.args[1])
				var = self.builder.icmp( llvm.core.ICMP_ULT, a, b, op.opname )
				var.name = op.result.name
				self.var_cache[ op.result ] = var

		if block is graph.returnblock:
			if block.inputargs[0] in self.allocas:
				stackvar = self.allocas[ block.inputargs[0] ]
				ret = self.builder.load( stackvar )
			else:
				ret = get( block.inputargs[0] )

			self.builder.ret( ret )


		for link in block.exits:
			print(tabs+'<LINK>	args:%s	exitcase:%s' %(link.args,link.exitcase))
			if link.target is block:
				print(tabs+'\t<WHILE LOOP> exit-switch: %s' %block.exitswitch)
				assert link.exitcase
				blk.name = 'while_loop'

				################################assert block.exitswitch in self.allocas


				con_bool = get( block.exitswitch )	# condition bool
				#print('condition bool',con_bool)
				assert len(block.exits) == 2
				assert block.exits.index( link ) == 1
				then_blk = blk

				else_block = block.exits[0].target
				if else_block not in self.blocks:
					if else_block is graph.returnblock:
						eblk = func.append_basic_block( 'else_return' )
					else:
						eblk = func.append_basic_block( 'else' )
					self.blocks[ else_block ] = eblk

				else_blk = self.blocks[ else_block ]
				else_blk.name = 'else_return'
				self.builder.position_at_end( then_blk )
				self.builder.cbranch( con_bool, then_blk, else_blk )


			elif link.target not in self.blocks:

				if len(block.exits)==1:
					if link.target is graph.returnblock:
						blk2 = func.append_basic_block( 'return' )
					else:
						blk2 = func.append_basic_block( 'block' )
					self.blocks[ link.target ] = blk2
					self.builder.position_at_end( blk )
					self.builder.branch( blk2 )

				self.flow( graph, block=link.target, indent=indent+1 )

	################################################
	def apply_hints(self, graph ):
		print(graph._llvm_hints)
		hints = graph._llvm_hints
		self.types['Signed'] = llvm.core.Type.int( 32 )		# restore
		self.types['Signed']._type = 'int32'

		if 'sizeof' in hints:
			if 'int' in hints['sizeof']:
				bits = hints['sizeof']['int']
				self.types['Signed'] = llvm.core.Type.int( bits )
				self.types['Signed']._type = 'int%s'%bits

	def make_llvm_function( self, graph ):
		var = graph.returnblock.inputargs[0]
		ret = self.llvm_type(var)
		args = []
		for arg in graph.startblock.inputargs:
			args.append( self.llvm_type( arg ) )
		ftype = llvm.core.Type.function( ret, args )
		func = llvm.core.Function.new(
			self.module,		# MODULE
			ftype,			# Function TYpe
			graph.name,		# Name
		)
		func.calling_convention = llvm.core.CC_C
		self.functions[ graph.name ] = func
		func._return_type = ret
		func._pypy_graph = graph
		return func

	def __init__(self, graphs, optimize=2):
		import pypy.objspace.flow.model
		self.optimize = optimize
		self.functions = {}

		self.module = llvm.core.Module.new( 'myjit' )
		self.engine = llvm.ee.ExecutionEngine.new( self.module )
		self.pman = llvm.passes.FunctionPassManager.new(self.module)

		# Set up the optimizer pipeline. Start with registering info about how the target lays out data structures.
		self.pman.add( self.engine.target_data )
		if self.optimize:
			self.pman.add( llvm.passes.PASS_PROMOTE_MEMORY_TO_REGISTER )
		if self.optimize >= 2:
			print('DOING ADVANCED PASS OPTS')
			# Do simple "peephole" optimizations and bit-twiddling optzns.
			self.pman.add( llvm.passes.PASS_INSTRUCTION_COMBINING)
			# Reassociate expressions.
			self.pman.add( llvm.passes.PASS_REASSOCIATE)
			# Eliminate Common SubExpressions.
			self.pman.add( llvm.passes.PASS_GVN)
			# Simplify the control flow graph (deleting unreachable blocks, etc).
			self.pman.add( llvm.passes.PASS_CFG_SIMPLIFICATION)
		if self.optimize >= 3:
			self.pman.add( llvm.passes.PASS_LCSSA)
			## LICM is smart - moves what ops it can outside of loops to entry
			self.pman.add( llvm.passes.PASS_LICM)
			self.pman.add( llvm.passes.PASS_LOOP_DEPENDENCE_ANALYSIS)
			self.pman.add( llvm.passes.PASS_LOOP_EXTRACTOR)
			self.pman.add( llvm.passes.PASS_LOOP_SIMPLIFY)
			self.pman.add( llvm.passes.PASS_LOOP_UNROLL)
			self.pman.add( llvm.passes.PASS_LOOP_UNSWITCH)

		self.pman.initialize()


		for graph in graphs:
			assert isinstance(graph, pypy.objspace.flow.model.FunctionGraph)
			root = Block( graph.startblock, graph=graph )
			root.show()
			self.apply_hints( graph )
			llfunc = self.make_llvm_function( graph )
			root.setup_llvm_translation( llfunc )
			Block.types = self.types
			root.generate_llvm_ir()


			if DEBUG:
				print('============= llvm asm (RAW) ==============')
				print(llfunc)
				print('-'*80)
			if self.optimize:
				self.pman.run( llfunc )
				if DEBUG:
					print('============= llvm asm (OPTIMIZE LEVEL:%s)=============='%self.optimize)
					print(llfunc)
					print('-'*80)


	def __init_old__(self, graphs, optimize=2):
		import pypy.objspace.flow.model
		self.optimize = optimize
		self.functions = {}

		self.module = llvm.core.Module.new( 'myjit' )
		self.engine = llvm.ee.ExecutionEngine.new( self.module )
		self.pman = llvm.passes.FunctionPassManager.new(self.module)

		# Set up the optimizer pipeline. Start with registering info about how the
		# target lays out data structures.
		self.pman.add( self.engine.target_data )
		if self.optimize >=10:
			print('DOING CRAZY PASS OPTS')
			for i in range(88):
				opt = i+1
				if opt in (4,5,6,12,15,19,20,25,26,27,29,30,31,37,38,54,57,60,61,62,68,71,82,83,84,85): continue
				self.pman.add( opt )
		else:
			if self.optimize:
				self.pman.add( llvm.passes.PASS_PROMOTE_MEMORY_TO_REGISTER )
			if self.optimize >= 2:
				print('DOING ADVANCED PASS OPTS')
				# Do simple "peephole" optimizations and bit-twiddling optzns.
				self.pman.add( llvm.passes.PASS_INSTRUCTION_COMBINING)
				# Reassociate expressions.
				self.pman.add( llvm.passes.PASS_REASSOCIATE)
				# Eliminate Common SubExpressions.
				self.pman.add( llvm.passes.PASS_GVN)
				# Simplify the control flow graph (deleting unreachable blocks, etc).
				self.pman.add( llvm.passes.PASS_CFG_SIMPLIFICATION)
			if self.optimize >= 3:
				self.pman.add( llvm.passes.PASS_LCSSA)
				## LICM is smart - moves what ops it can outside of loops to entry
				self.pman.add( llvm.passes.PASS_LICM)
				self.pman.add( llvm.passes.PASS_LOOP_DEPENDENCE_ANALYSIS)
				self.pman.add( llvm.passes.PASS_LOOP_EXTRACTOR)
				self.pman.add( llvm.passes.PASS_LOOP_SIMPLIFY)

				#self.pman.add( llvm.passes.PASS_LOOP_DELETION)
				#self.pman.add( llvm.passes.PASS_LOOP_INDEX_SPLIT)
				#self.pman.add( llvm.passes.PASS_LOOP_ROTATE)

				self.pman.add( llvm.passes.PASS_LOOP_UNROLL)
				self.pman.add( llvm.passes.PASS_LOOP_UNSWITCH)

		self.pman.initialize()


		for graph in graphs:
			assert isinstance(graph, pypy.objspace.flow.model.FunctionGraph)
			if DEBUG: print('============JIT: %s' %graph)
			self._instance_vars = {}

			print(graph._llvm_hints)
			hints = graph._llvm_hints
			self.types['Signed'] = llvm.core.Type.int( 32 )		# restore
			self.types['Signed']._type = 'int32'

			if 'sizeof' in hints:
				if 'int' in hints['sizeof']:
					bits = hints['sizeof']['int']
					self.types['Signed'] = llvm.core.Type.int( bits )
					self.types['Signed']._type = 'int%s'%bits

			var = graph.returnblock.inputargs[0]
			ret = self.llvm_type(var)
			args = []
			for arg in graph.startblock.inputargs:
				args.append( self.llvm_type( arg ) )
			ftype = llvm.core.Type.function( ret, args )
			func = llvm.core.Function.new(
				self.module,
				ftype,
				graph.name,
			)
			func.calling_convention = llvm.core.CC_C
			self.functions[ graph.name ] = func
			func._return_type = ret

			self.var_cache = vmap = {}
			for i,pypyarg in enumerate(graph.startblock.inputargs):	# assume Variables
				var = func.args[ i ]
				var.name = pypyarg.name
				vmap[ pypyarg ] = var
				for link in graph.startblock.exits:
					if pypyarg in link.args:
						arg2 = link.target.inputargs[ link.args.index(pypyarg) ]
						vmap[ arg2 ] = var

			self.builder = None	# only one builder
			self.blocks = {}	# pypy-block : llvm-block
			self.flow( graph )
			print(func)

			if DEBUG:
				print('============= llvm asm (RAW) ==============')
				print(func)
				print('-'*80)
			if self.optimize:
				self.pman.run( func )
				if DEBUG:
					print('============= llvm asm (OPTIMIZE LEVEL:%s)=============='%self.optimize)
					print(func)
					print('-'*80)

		self.module.verify()

	def call( self, func_name, *args ):
		llargs = []
		for arg in args:
			if type(arg) is int:
				llarg = llvm.ee.GenericValue.int(
					self.types['Signed'],
					arg
				)
				llargs.append( llarg )
			elif type(arg) is float:
				llarg = llvm.ee.GenericValue.real(
					self.types['Float'],
					arg
				)
				llargs.append( llarg )

			else:
				raise NotImplementedError

		func = self.functions[ func_name ]
		retval = self.engine.run_function( func, llargs )
		_type = func._return_type._type
		if _type in ('int32','int64'):
			return retval.as_int()	# TODO Signed INT
		elif _type=='float32':
			return retval.as_real( self.types['Float'] )
		elif _type=='float64':
			return retval.as_real( self.types['Double'] )
		else:
			raise NotImplemented


	def get_wrapper_function(self, func_name, use_ctypes=False):
		if not use_ctypes:
			lamb = eval('lambda *args: self.call("%s", *args)'%func_name, {'self':self})
			return lamb
		else:
			llfunc = self.functions[ func_name ]
			llfunc.does_not_throw = True		# add nounwind attribute to function
			llfunc_pointer = self.engine.get_pointer_to_function( llfunc )
			graph = llfunc._pypy_graph
			FUNC_TYPE = ctypes.CFUNCTYPE(
				self.ctypes_type( graph.returnblock.inputargs[0] ), 
				*[ self.ctypes_type(arg) for arg in graph.startblock.inputargs ]
			)
			return FUNC_TYPE( llfunc_pointer )


	CONCRETE_CTYPES = {
		'Bool'	: ctypes.c_bool,
		'Void'	: ctypes.c_void_p,
		'Signed'	: ctypes.c_int32,
		'Float'	: ctypes.c_float,		# 32bits
		'Double'	: ctypes.c_double,	# 64bits
	}

	def ctypes_type( self, var ):
		return self.CONCRETE_CTYPES[ var.concretetype._name ]


