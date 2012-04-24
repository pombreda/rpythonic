# Rpython LLVM JIT - April 20, 2012
# by Brett and The Blender Research Lab.
# License: BSD
import llvm.core
import llvm.ee
import llvm.passes
import sys

DEBUG = '--debug' in sys.argv

######################################################################
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
			return llvm.core.Type.pointer( llvm.core.Type.void() )
		else:
			return self.types[ ctype._name ]

	def get_var_or_const( self, pypyarg ):
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
				return llvm.core.Constant.real( self.types['Float'], pypyarg.value )
			else:
				return llvm.core.Constant.int( self.types['Signed'], pypyarg.value )

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
					llvm.core.Constant.int( self.types['Signed'], index )
				)
		return var


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
							stackvar = self.builder.alloca( self.llvm_type(arg), 'st_'+arg.name )
							if link.prevblock is graph.startblock and len(func.args):	 # TODO FIXME
								self.builder.store( func.args[link.args.index(arg)], stackvar )
						else:	# Constant
							stackvar = self.builder.alloca( self.llvm_type(arg), 'st' )
							const = llvm.core.Constant.int( self.types['Signed'], arg.value )
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
		print(tabs+repr(block)+str(block.inputargs))

		get = self.get_var_or_const

		instance_vars = {}
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
				if op.args[0] in instance_vars:
					instance_vars[ op.result ] = get(op.args[0])


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
					instance_vars[ gcop.result  ] = var

				elif fname == '__getitem__':
					a = instance_vars[op.args[1]]
					b = get(op.args[2])
					var = self.builder.extract_element( a,b, 'element' )
					self.var_cache[ op.result ] = var
					if op.result in self.allocas:
						self.builder.store( var, self.allocas[op.result] )

				elif fname == '__setitem__':
					inst = instance_vars[op.args[1]]
					idx = get(op.args[2])
					val = get(op.args[3])
					var = self.builder.insert_element( inst, val, idx )
					self.var_cache[ op.result ] = var

				else:
					assert 0

			elif op.opname == 'int_add':
				if op.args[0] in block.inputargs and op.args[0] in self.allocas:
					a = self.builder.load( self.allocas[op.args[0]], op.args[0].name )
				else:
					a = get(op.args[0])

				if op.args[1] in block.inputargs:
					b = self.builder.load( self.allocas[op.args[1]], op.args[1].name )
				else:
					b = get(op.args[1])

				var = self.builder.add( a, b, op.opname )
				var.name = op.result.name
				if op.result in self.allocas:
					self.builder.store( var, self.allocas[op.result] )
					#var = self.allocas[op.result]	# ugly

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



	def __init__(self, graphs, optimize=2):
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


	def get_wrapper_function(self, func_name):
		lamb = eval('lambda *args: self.call("%s", *args)'%func_name, {'self':self})
		return lamb


