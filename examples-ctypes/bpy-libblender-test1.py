#!/usr/bin/python3
#!../../pypy-1.6/bin/pypy
'''
PyPy1.6 fails:

/home/brett/.rpythonic/clibs/linux32/libbpy.so
Traceback (most recent call last):
  File "app_main.py", line 53, in run_toplevel
  File "./bpy-libblender-test1.py", line 12, in <module>
    blender = rpythonic.module( 'blender' )
  File "../rpythonic/rpythonic.py", line 375, in module
    mod = load( name, mode, platform )
  File "../rpythonic/rpythonic.py", line 324, in load
    if debug: mod = _load( name, mode, platform )
  File "../rpythonic/rpythonic.py", line 319, in _load
    fromlist=[name]
  File "/home/brett/.rpythonic/genctypes/blender/__init__.py", line 7672, in <module>
    ( "toolsettings", ctypes.POINTER(ToolSettings) ),
  File "/home/brett/pypy-1.6/lib_pypy/_ctypes/pointer.py", line 179, in POINTER
    {'_type_': cls})
  File "/home/brett/pypy-1.6/lib_pypy/_ctypes/pointer.py", line 31, in __new__
    self.set_type(obj, typedict['_type_'])
  File "/home/brett/pypy-1.6/lib_pypy/_ctypes/pointer.py", line 70, in set_type
    self._ffiargtype = _ffi.types.Pointer(TP.get_ffi_argtype())
  File "/home/brett/pypy-1.6/lib_pypy/_ctypes/basics.py", line 57, in get_ffi_argtype
    return _shape_to_ffi_type(self._ffiargshape)
  File "/home/brett/pypy-1.6/lib_pypy/_ctypes/basics.py", line 200, in _shape_to_ffi_type
    return shape[0].get_ffi_type()
MemoryError

'''

import os,sys, time, ctypes
from random import *

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic

sys.path.append( '../../bpybuild/bin/' )

blender = rpythonic.module( 'blender' )
assert blender
print( blender )

#bmain = ctypes.POINTER(ctypes.c_void_p).from_address(ctypes.addressof(blend_cdll.G)).contents.value
#ctr = blend_cdll.BPy_GetContext()
#sce = bpy.data.scenes[scene]
#ctr = blend_cdll.BPy_GetContext()
#blend_cdll.ED_screen_set_scene(ctr, sce.as_pointer())


import bpy
print( bpy )

ptr = ctypes.pointer( ctypes.c_void_p(bpy.context.as_pointer()) )
evil_C = ctypes.POINTER(ctypes.c_void_p).from_address( bpy.context.as_pointer() )
#C = blender.Context( pointer=ctypes.pointer(ptr) )

_argv = ''
for arg in sys.argv: _argv += arg + ' '
_argv = bytes( _argv, 'utf-8' )

argc = len(sys.argv)
argv = ctypes.pointer(ctypes.c_char_p(_argv))

ba = blender.BLI_argsInit(argc, argv)
#blender.setupArguments(C, ba, syshandle)
blender.BLI_argsParse(ba, 1, None, None)  # required, segfaults without this
blender.WM_init( evil_C, argc, argv )

def init():


	C = blender.CTX_create()
	print( C, dir(C) )

	blender.BLI_threadapi_init()
	blender.RNA_init()
	blender.RE_engines_init()
	#blender.pluginapi_force_ref()
	blender.init_nodesystem()
	blender.initglobals()
	blender.IMB_init()
	#syshandle = blender.SYS_GetSystem()
	#blender.GEN_init_messaging_system()
	ba = blender.BLI_argsInit(argc, argv)
	#blender.setupArguments(C, ba, syshandle)
	blender.BLI_argsParse(ba, 1, None, None)  # required, segfaults without this
	blender.sound_init_once()
	blender.init_def_material()

	C.wm_ghost_init()

	C.WM_init( argc, argv )

