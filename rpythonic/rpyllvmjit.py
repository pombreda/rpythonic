# Rpython LLVM JIT - April 20, 2012
# by Brett and The Blender Research Lab.
# License: BSD
import llvm.core
import llvm.ee
import llvm.passes


######################################################################
######################################################################
class JIT(object):
	types = {
		'Bool'	: llvm.core.Type.int(32),	# is this correct?
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
			#return self.types[ 'Signed' ]
		else:
			return self.types[ ctype._name ]

	def __init__(self, graphs, optimize=True):
		#import pypy.rpython.lltypesystem.lltype
		self.optimize = optimize
		self.functions = {}

		self.module = llvm.core.Module.new( 'myjit' )
		self.engine = llvm.ee.ExecutionEngine.new( self.module )
		self.pman = llvm.passes.FunctionPassManager.new(self.module)

		# Set up the optimizer pipeline. Start with registering info about how the
		# target lays out data structures.
		self.pman.add( self.engine.target_data )
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
			print('============JIT: %s' %graph)

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

			vmap = {}
			for i,arg in enumerate(graph.startblock.inputargs):
				var = func.args[ i ]
				var.name = arg.name
				vmap[ arg ] = var

			def get( _arg, _cache ):
				if _arg in _cache:	# assume variable
					v = _cache[ _arg ]
					v.name = _arg.name
					return v
				else:			# assume constant
					return llvm.core.Constant.int( self.types['Signed'], _arg.value )

			blk = func.append_basic_block( 'entry' )
			builder = llvm.core.Builder.new( blk )
			for op in graph.startblock.operations:
				print(op)
				if op.opname == 'direct_call' and len(op.args)==3:
					var = builder.malloc( self.llvm_type(op.result) )
					var.name = op.result.name
					vmap[ op.result ] = var
				elif op.opname == 'int_add':
					a = get(op.args[0],vmap)
					b = get(op.args[1],vmap)
					var = builder.add( a, b, op.opname )
					var.name = op.result.name
					vmap[ op.result ] = var

			emap = {}
			for link in graph.startblock.exits:
				for i,arg in enumerate(link.args):
					var = vmap[ arg ]
					emap[ link.target.inputargs[i] ] = var

			if graph.returnblock.inputargs:
				print( graph.returnblock.inputargs )
				arg = graph.returnblock.inputargs[0]
				#if arg in vmap:
				builder.ret( emap[arg] )
			else: assert 0

			print('============= raw llvm asm ==============')
			print(func)
			print('-'*80)
			if self.optimize:
				self.pman.run( func )
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
		func = self.functions[ func_name ]
		retval = self.engine.run_function( func, llargs )
		return retval.as_int()

