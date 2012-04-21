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
	types = {
		'Bool'	: llvm.core.Type.int(1),
		'Void'	: llvm.core.Type.void(),
		'Signed'	: llvm.core.Type.int(32),
	}
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
			return llvm.core.Constant.int( self.types['Signed'], pypyarg.value )

	def find_cached_var(self, var): assert 0


	def flow( self, graph, block=None, indent=0 ):
		func = self.functions[ graph.name ]
		if not block:
			assert not self.blocks and not self.builder
			block = graph.startblock
			blk = func.append_basic_block( 'entry' )
			self.builder = llvm.core.Builder.new( blk )
			self.blocks[ block ] = blk
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

		for op in block.operations:
			print( ('\t'*indent)+str(op) )
			if op.opname == 'int_add':
				a = get(op.args[0])
				b = get(op.args[1])
				var = self.builder.add( a, b, op.opname )
				var.name = op.result.name
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
			elif op.opname == 'same_as':	# can fold
				self.var_cache[ op.result ] = get(op.args[0])

		if block is graph.returnblock:
			ret = get( block.inputargs[0] )
			self.builder.ret( ret )


		for link in block.exits:
			print(tabs+'<LINK>	args:%s	exitcase:%s' %(link.args,link.exitcase))
			if link.target is block:
				print(tabs+'\t<WHILE LOOP> exit-switch: %s' %block.exitswitch)
				assert link.exitcase
				blk.name = 'while_loop'
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



	def __init__(self, graphs, optimize=True):
		import pypy.objspace.flow.model
		self.optimize = optimize
		self.functions = {}

		self.module = llvm.core.Module.new( 'myjit' )
		self.engine = llvm.ee.ExecutionEngine.new( self.module )
		self.pman = llvm.passes.FunctionPassManager.new(self.module)

		# Set up the optimizer pipeline. Start with registering info about how the
		# target lays out data structures.
		self.pman.add( self.engine.target_data )
		self.pman.add( llvm.passes.PASS_PROMOTE_MEMORY_TO_REGISTER )
		# Do simple "peephole" optimizations and bit-twiddling optzns.
		self.pman.add( llvm.passes.PASS_INSTRUCTION_COMBINING)
		# Reassociate expressions.
		self.pman.add( llvm.passes.PASS_REASSOCIATE)
		# Eliminate Common SubExpressions.
		self.pman.add( llvm.passes.PASS_GVN)
		# Simplify the control flow graph (deleting unreachable blocks, etc).
		self.pman.add( llvm.passes.PASS_CFG_SIMPLIFICATION)
		self.pman.initialize()


		for graph in graphs:
			assert isinstance(graph, pypy.objspace.flow.model.FunctionGraph)
			if DEBUG: print('============JIT: %s' %graph)

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
				print('============= raw llvm asm ==============')
				print(func)
				print('-'*80)
			if self.optimize:
				self.pman.run( func )
				if DEBUG:
					print('============= OPT llvm asm ==============')
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
			else:
				raise NotImplementedError

		func = self.functions[ func_name ]
		retval = self.engine.run_function( func, llargs )
		return retval.as_int()		# TODO return other types!

