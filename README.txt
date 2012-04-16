RPythonic: Beta7 - April, 2012
Copyright: Brett Hartshorn (bhartsho@yahoo.com)
License: BSD

INSTALLING:
	apt-get install python-ply libreadline-dev

	Optional: 
		java
		android SDK/NDK
		llvm and llvm-gcc


DEV NOTES:
	C++ Wrapper Gen - in progress...
	see the "scripts" folder for wrapper generator examples


PYPY NOTES:
	A stripped down PyPy is now included with Rpythonic,
	it is no longer required you fetch the PyPy source and patch it.

	Custom Hacks:
		copy "pypy-trunk/pypy" to "rpythonic"

		pypy/translator/c/debug_print.c
			line 92, remove the ifndef block using "clock_gettime"

		copy file "opcode.py" from "lib-python/modified-2.7" to "pypy/tool"

		pypy/tool/stdlib_opcode.py
			line 108:		
			def load_pypy_opcode():
			    #from pypy.tool.lib_pypy import LIB_PYTHON_MODIFIED
			    #opcode_path = LIB_PYTHON_MODIFIED.join('opcode.py')
			    import os
			    opcode_path = os.path.join( os.path.split(os.path.abspath(__file__))[0], 'opcode.py' )

		from "pypy-trunk/" copy "py" and "lib_pypy" to "rpythonic"

		from "pypy-trunk/" copy "pytest.py" and "_pytest" to "rpythonic/pypy"
			modify pytest.py:
				import os
				DIR = os.path.split(os.path.abspath(__file__))[0]
				if DIR not in sys.path: sys.path.append( DIR )



