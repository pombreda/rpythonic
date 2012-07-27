#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic
#rpythonic.set_cache( '../cache' )
rpythonic.set_pypy_root( '../../pypy' )
################################


## requires Android SDK ##
SDK_ROOT = '../../android-sdk-linux_x86/'
assert os.path.isdir( SDK_ROOT )
SDK_TOOLS = os.path.join( SDK_ROOT, 'tools' )
## requires Android NDK
NDK_PLATFORM_VERSION = 'android-9'	# change this to match your NDK version
NDK_ROOT = '../../android-ndk-r5b/'	# change this to point to your NDK installation
assert os.path.isdir( NDK_ROOT )
NDK_ROOT = os.path.abspath( NDK_ROOT )
NDK_PLATFORM = os.path.join( NDK_ROOT, 'platforms/%s/arch-arm/' %NDK_PLATFORM_VERSION )
assert os.path.isdir( NDK_PLATFORM )
NDK_LIBS = os.path.join( NDK_PLATFORM, 'usr/lib/' )
assert os.path.isdir( NDK_LIBS )

USR = os.path.join( NDK_PLATFORM, 'usr')
USR_INC = os.path.join( USR, 'include' )
USR_LIB = os.path.join( USR, 'lib' )



#for name in 'android EGL GLES GLES2 SLES'.split():

## for stdarg.h
includes = [ 
	os.path.join( NDK_ROOT, 'toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/lib/gcc/arm-eabi/4.4.0/include/' )
]
defines = ['__ANDROID__']

name = 'android'
header = os.path.join(os.path.join( USR_INC, name ), 'native_activity.h')
library = os.path.join( USR_LIB, 'libandroid.so' )
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	ctypes=False, rffi=True, platform='android'
	)


name = 'EGL'
header = os.path.join(os.path.join( USR_INC, name ), 'egl.h')
library = os.path.join( USR_LIB, 'libEGL.so' )
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines,
	ctypes=False, rffi=True, platform='android'
	)

name = 'GLES'
header = os.path.join(os.path.join( USR_INC, name ), 'gl.h')
library = os.path.join( USR_LIB, 'libGLESv1_CM.so' )
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines,
	ctypes=False, rffi=True, platform='android'
	)

name = 'GLES2'
header = os.path.join(os.path.join( USR_INC, name ), 'gl2.h')
library = os.path.join( USR_LIB, 'libGLESv2.so' )
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines,
	ctypes=False, rffi=True, platform='android'
	)

name = 'SLES'
header = os.path.join(os.path.join( USR_INC, name ), 'OpenSLES.h')
library = os.path.join( USR_LIB, 'libOpenSLES.so' )
rpythonic.wrap( 
	name, header=header, library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines,
	ctypes=False, rffi=True, platform='android'
	)


name = 'android_native_app_glue'
header = os.path.join( NDK_ROOT, 'sources/android/native_app_glue/android_native_app_glue.h' )
rpythonic.wrap( 
	name, header=header, #library=library, 
	includes=includes, system_include=USR_INC,
	defines=defines,
	ctypes=False, rffi=True, platform='android'
	)

